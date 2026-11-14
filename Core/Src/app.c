/**
  ******************************************************************************
  * @file    app.c
  * @brief   定位、回零、限位报警与 OLED 界面
  *
  *  回零流程：向负方向慢速找 PB0(IN8 近端) -> 停 -> 等 120ms ->
  *            反向退 2.0 度 -> 该点设为 0 度。
  ******************************************************************************
  */

#include <string.h>

#include "main.h"
#include "cli.h"
#include "app.h"
#include "stepper.h"
#include "OLED.h"

#define LED_GPIO              GPIOA
#define LED_PIN               GPIO_PIN_5

/* 电机 : 转台 = 1 : 90，对外的角度、转速全部按“转台”计算 */
#define GEAR_RATIO            90u

#define APP_V_MIN             1u      /* 转台 0.1 rpm */
#define APP_V_MAX             80u     /* 转台 8.0 rpm */

#define HOME_RPM10            900u    /* 回零用电机转速 90 rpm = 转台 1 rpm */
#define HOME_BACKOFF_DEG10    10      /* 退让 1.0 度（转台角度） */
#define HOME_TIMEOUT_MS       120000u /* 找零超时 */
#define HOME_SETTLE_MS        120u    /* 撞到限位后等待机械回弹 */

#define F_NEAR                0x01u
#define F_FAR                 0x02u
#define F_HOME                0x04u

#define HOME_PHASE_NONE       0u
#define HOME_PHASE_SEEK       1u
#define HOME_PHASE_SETTLE     2u
#define HOME_PHASE_BACKOFF    3u

static AppState s_state = APP_READY;
static uint8_t  s_fault = 0u;

static uint8_t  s_home_phase = HOME_PHASE_NONE;
static uint32_t s_home_tick  = 0u;
static uint32_t s_home_rpm   = 60u;

static uint32_t s_table_rpm10 = 10u;   /* 转台转速 1.0 rpm */

static void ui_task(void);
static void led_task(void);
static void home_fail(void);

/* -------------------------------------------------------------------------- */
/* 减速比换算（对外角度单位 0.1 度，指转台角度）                                */
/* -------------------------------------------------------------------------- */
static int32_t table_deg10_to_steps(int32_t deg10)
{
  int64_t v = (int64_t)deg10 * (int64_t)GEAR_RATIO * (int64_t)Stepper_GetStepsPerRev();

  v = (v >= 0) ? (v + 1800) / 3600 : (v - 1800) / 3600;
  return (int32_t)v;
}

static int32_t steps_to_table_deg10(int32_t steps)
{
  int64_t den = (int64_t)GEAR_RATIO * (int64_t)Stepper_GetStepsPerRev();
  int64_t v   = (int64_t)steps * 3600;

  v = (v >= 0) ? (v + den / 2) / den : (v - den / 2) / den;
  return (int32_t)v;
}

/* -------------------------------------------------------------------------- */
/* 小工具                                                                     */
/* -------------------------------------------------------------------------- */
static void fmt_rpm(char *out, uint32_t rpm10)
{
  char     tmp[8];
  uint8_t  n = 0u;
  uint32_t ip = rpm10 / 10u;
  uint32_t fp = rpm10 % 10u;

  do
  {
    tmp[n++] = (char)('0' + (ip % 10u));
    ip /= 10u;
  } while ((ip != 0u) && (n < 6u));

  while (n > 0u)
  {
    *out++ = tmp[--n];
  }
  *out++ = '.';
  *out++ = (char)('0' + fp);
  *out = '\0';
}

static char *str_put(char *dst, const char *src)
{
  while (*src != '\0')
  {
    *dst++ = *src++;
  }
  return dst;
}

/* -------------------------------------------------------------------------- */
/* 状态                                                                       */
/* -------------------------------------------------------------------------- */
const char *App_StateStr(void)
{
  switch (s_state)
  {
    case APP_MOVING: return "MOVE";
    case APP_HOMING: return "HOME";
    case APP_FAULT:  return "FAULT";
    default:         return "READY";
  }
}

AppState App_GetState(void)
{
  return s_state;
}

uint8_t App_GetFault(void)
{
  return s_fault;
}

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
void App_Init(void)
{
  s_state      = APP_READY;
  s_fault      = 0u;
  s_home_phase = HOME_PHASE_NONE;

  /* 默认转台转速 1.0 rpm（电机 90 rpm），A 指令按转台角度走 */
  s_table_rpm10 = 10u;
  Stepper_SetSpeedRpm10(s_table_rpm10 * GEAR_RATIO);

  OLED_Init();
  OLED_Clear();
  ui_task();

  CLI_Print("\r\n== STEP TABLE CTRL ==\r\n");
  CLI_Print("H help   ? status\r\n");
  App_PrintStatus();
}

/* -------------------------------------------------------------------------- */
/* 运动指令                                                                   */
/* -------------------------------------------------------------------------- */
bool App_MoveAbsDeg10(int32_t deg10)
{
  int32_t target;
  int32_t pos;

  if ((s_state == APP_FAULT) || (s_state == APP_HOMING))
  {
    return false;
  }

  pos    = Stepper_GetPosSteps();
  target = table_deg10_to_steps(deg10);

  /* 软件只做方向互锁，硬限位由去抖逻辑刹车 */
  if ((target < pos) && Stepper_LimitNearActive())
  {
    return false;
  }
  if ((target > pos) && Stepper_LimitFarActive())
  {
    return false;
  }

  if (!Stepper_MoveToSteps(target))
  {
    return false;
  }

  s_state = APP_MOVING;
  return true;
}

bool App_MoveRelDeg10(int32_t deg10)
{
  int32_t target;
  int32_t pos;

  if ((s_state == APP_FAULT) || (s_state == APP_HOMING))
  {
    return false;
  }

  pos    = Stepper_GetPosSteps();
  target = pos + table_deg10_to_steps(deg10);

  if ((target < pos) && Stepper_LimitNearActive())
  {
    return false;
  }
  if ((target > pos) && Stepper_LimitFarActive())
  {
    return false;
  }

  if (!Stepper_MoveToSteps(target))
  {
    return false;
  }

  s_state = APP_MOVING;
  return true;
}

bool App_Jog(int8_t dir)
{
  if ((s_state == APP_FAULT) || (s_state == APP_HOMING))
  {
    return false;
  }
  if (!Stepper_Jog(dir))
  {
    return false;
  }
  s_state = APP_MOVING;
  return true;
}

/**
  * @brief  设置转台转速（单位 0.1 rpm），内部换算成电机转速
  */
bool App_SetSpeedRpm10(uint32_t rpm10)
{
  if ((rpm10 < APP_V_MIN) || (rpm10 > APP_V_MAX))
  {
    return false;
  }

  s_table_rpm10 = rpm10;
  Stepper_SetSpeedRpm10(rpm10 * GEAR_RATIO);
  return true;
}

uint32_t App_GetSpeedRpm10(void)
{
  return s_table_rpm10;
}

bool App_SetSpr(uint32_t spr)
{
  if ((spr < STEPPER_SPR_MIN) || (spr > STEPPER_SPR_MAX))
  {
    return false;
  }
  Stepper_SetStepsPerRev(spr);
  return true;
}

/* -------------------------------------------------------------------------- */
/* 停止 / 清零 / 清报警                                                        */
/* -------------------------------------------------------------------------- */
void App_Stop(void)
{
  Stepper_Stop();

  if (s_home_phase != HOME_PHASE_NONE)
  {
    Stepper_SetSpeedRpm10(s_home_rpm);
    s_home_phase = HOME_PHASE_NONE;
  }

  if (s_fault != 0u)
  {
    s_state = APP_FAULT;
  }
  else
  {
    s_state = APP_READY;
  }
}

void App_Zero(void)
{
  Stepper_Zero();
  s_home_phase = HOME_PHASE_NONE;

  if (s_fault == 0u)
  {
    s_state = APP_READY;
  }
}

void App_ClearFault(void)
{
  s_fault = 0u;
  if (s_state == APP_FAULT)
  {
    s_state = APP_READY;
  }
}

/* -------------------------------------------------------------------------- */
/* 回零                                                                       */
/* -------------------------------------------------------------------------- */
void App_Home(void)
{
  if (s_state == APP_HOMING)
  {
    return;
  }

  Stepper_Stop();
  s_fault &= (uint8_t)~(F_NEAR | F_FAR | F_HOME);
  s_home_rpm   = Stepper_GetSpeedRpm10();
  s_home_phase = HOME_PHASE_SEEK;
  s_home_tick  = HAL_GetTick();
  s_state      = APP_HOMING;

  Stepper_SetSpeedRpm10(HOME_RPM10);

  if (Stepper_LimitNearActive())
  {
    /* 已经压着近端限位，直接退让 */
    s_home_phase = HOME_PHASE_SETTLE;
    s_home_tick  = HAL_GetTick();
  }
  else if (!Stepper_Jog(-1))
  {
    home_fail();
    return;
  }

  CLI_Print("OK HOMING\r\n");
}

static void home_fail(void)
{
  Stepper_Stop();
  Stepper_SetSpeedRpm10(s_home_rpm);
  s_fault |= F_HOME;
  s_state      = APP_FAULT;
  s_home_phase = HOME_PHASE_NONE;
  CLI_Print("!HOME FAIL\r\n");
}

static void home_task(uint8_t limit_event)
{
  int32_t back;

  switch (s_home_phase)
  {
    case HOME_PHASE_SEEK:
      if (limit_event == 1u)
      {
        Stepper_Stop();
        s_home_phase = HOME_PHASE_SETTLE;
        s_home_tick  = HAL_GetTick();
      }
      else if (limit_event == 2u)
      {
        home_fail();
      }
      else if ((HAL_GetTick() - s_home_tick) > HOME_TIMEOUT_MS)
      {
        home_fail();
      }
      else
      {
        /* 继续找 */
      }
      break;

    case HOME_PHASE_SETTLE:
      if ((HAL_GetTick() - s_home_tick) >= HOME_SETTLE_MS)
      {
        back = table_deg10_to_steps(HOME_BACKOFF_DEG10);
        if (back < 1)
        {
          back = 1;
        }
        if (Stepper_MoveSteps(back))
        {
          s_home_phase = HOME_PHASE_BACKOFF;
        }
        else
        {
          home_fail();
        }
      }
      break;

    case HOME_PHASE_BACKOFF:
      if (limit_event != 0u)
      {
        home_fail();
      }
      else if (!Stepper_IsBusy())
      {
        Stepper_Zero();
        Stepper_SetSpeedRpm10(s_home_rpm);
        s_state      = APP_READY;
        s_home_phase = HOME_PHASE_NONE;
        CLI_Print("DONE HOME POS +000.0\r\n");
      }
      else
      {
        /* 退让中 */
      }
      break;

    default:
      s_home_phase = HOME_PHASE_NONE;
      break;
  }
}

/* -------------------------------------------------------------------------- */
/* 状态打印                                                                   */
/* -------------------------------------------------------------------------- */
void App_PrintStatus(void)
{
  char    buf[96];
  char    *p = buf;
  char    num[16];

  p = str_put(p, "ST=");
  p = str_put(p, App_StateStr());

  p = str_put(p, " POS=");
  Fmt_Fix1(num, steps_to_table_deg10(Stepper_GetPosSteps()));
  p = str_put(p, num);

  p = str_put(p, " TGT=");
  Fmt_Fix1(num, steps_to_table_deg10(Stepper_GetTargetSteps()));
  p = str_put(p, num);

  p = str_put(p, " V=");
  fmt_rpm(num, App_GetSpeedRpm10());
  p = str_put(p, num);

  p = str_put(p, " SPR=");
  Fmt_Int(num, (int32_t)Stepper_GetStepsPerRev());
  p = str_put(p, num);

  p = str_put(p, " LIM=");
  p = str_put(p, Stepper_LimitNearActive() ? "1" : "0");
  p = str_put(p, "/");
  p = str_put(p, Stepper_LimitFarActive() ? "1" : "0");

  p = str_put(p, " RX=");
  Fmt_Int(num, (int32_t)CLI_GetRxCount());
  p = str_put(p, num);

  if (s_fault != 0u)
  {
    p = str_put(p, " FLT=");
    Fmt_Int(num, (int32_t)s_fault);
    p = str_put(p, num);
  }

  p = str_put(p, "\r\n");
  *p = '\0';
  CLI_Print(buf);
}

/* -------------------------------------------------------------------------- */
/* 主循环                                                                     */
/* -------------------------------------------------------------------------- */
#define LIMIT_DEBOUNCE_MS   10u

void App_Poll(void)
{
  static uint8_t  pending_lim = 0u;
  static uint32_t pending_t0  = 0u;
  uint8_t  confirmed = 0u;
  uint8_t  event;
  uint32_t now = HAL_GetTick();

  /* 限位沿先记下来，10ms 后仍为低才算真撞限位（滤掉悬空/干扰毛刺） */
  event = Stepper_TakeLimitEvent();
  if (event != 0u)
  {
    pending_lim = event;
    pending_t0  = now;
  }

  if ((pending_lim != 0u) && ((now - pending_t0) >= LIMIT_DEBOUNCE_MS))
  {
    uint8_t which = pending_lim;

    pending_lim = 0u;

    if (((which == 1u) && Stepper_LimitNearActive()) ||
        ((which == 2u) && Stepper_LimitFarActive()))
    {
      confirmed = which;
    }
  }

  if (s_state == APP_HOMING)
  {
    home_task(confirmed);
  }
  else if (confirmed == 1u)
  {
    Stepper_Stop();
    if ((s_fault & F_NEAR) == 0u)
    {
      CLI_Print("\r\n!LIMIT NEAR (PB0)\r\n");
    }
    s_fault |= F_NEAR;
    s_state = APP_FAULT;
  }
  else if (confirmed == 2u)
  {
    Stepper_Stop();
    if ((s_fault & F_FAR) == 0u)
    {
      CLI_Print("\r\n!LIMIT FAR (PB1)\r\n");
    }
    s_fault |= F_FAR;
    s_state = APP_FAULT;
  }
  else if ((s_state == APP_MOVING) && !Stepper_IsBusy())
  {
    char num[16];

    s_state = APP_READY;
    Fmt_Fix1(num, steps_to_table_deg10(Stepper_GetPosSteps()));
    CLI_Print("DONE POS ");
    CLI_Print(num);
    CLI_Print("\r\n");
  }
  else
  {
    /* 空 */
  }

  ui_task();
  led_task();
}

/* -------------------------------------------------------------------------- */
/* OLED 界面                                                                  */
/* -------------------------------------------------------------------------- */
/**
  * @brief  行宽只有 16 字符，角度超大时钳位，避免越界
  */
static int32_t ui_clamp_deg10(int32_t deg10)
{
  if (deg10 > 999999)
  {
    return 999999;
  }
  if (deg10 < -999999)
  {
    return -999999;
  }
  return deg10;
}

static void ui_build(char lines[4][17])
{
  uint8_t i;
  char    num[16];
  uint8_t n;

  for (i = 0u; i < 4u; i++)
  {
    (void)memset(lines[i], ' ', 16u);
    lines[i][16] = '\0';
  }

  /* 第 1 行：状态 + 转速 */
  {
    const char *st = App_StateStr();
    char rpm[10];

    for (i = 0u; (st[i] != '\0') && (i < 5u); i++)
    {
      lines[0][i] = st[i];
    }
    fmt_rpm(rpm, App_GetSpeedRpm10());
    n = (uint8_t)strlen(rpm);
    for (i = 0u; i < n; i++)
    {
      lines[0][13u - n + i] = rpm[i];
    }
    lines[0][13] = 'r';
    lines[0][14] = 'p';
    lines[0][15] = 'm';
  }

  /* 第 2 行：当前位置（转台角度） */
  Fmt_Fix1(num, ui_clamp_deg10(steps_to_table_deg10(Stepper_GetPosSteps())));
  (void)strcpy(&lines[1][0], "POS ");
  (void)strcpy(&lines[1][4], num);
  n = (uint8_t)strlen(num);
  (void)strcpy(&lines[1][4u + n], "deg");

  /* 第 3 行：目标位置（转台角度） */
  Fmt_Fix1(num, ui_clamp_deg10(steps_to_table_deg10(Stepper_GetTargetSteps())));
  (void)strcpy(&lines[2][0], "TGT ");
  (void)strcpy(&lines[2][4], num);
  n = (uint8_t)strlen(num);
  (void)strcpy(&lines[2][4u + n], "deg");

  /* 第 4 行：限位 + 每圈步数 */
  {
    char     tmp[18];
    char     k[8];
    uint32_t spr = Stepper_GetStepsPerRev();
    uint8_t  m = 0u;
    uint8_t  a;
    uint8_t  b;

    do
    {
      k[m++] = (char)('0' + (spr % 10u));
      spr /= 10u;
    } while ((spr != 0u) && (m < 6u));
    k[m] = '\0';

    /* 上面是低位在前，翻转成正序，否则 3200 会显示成 0023 */
    for (a = 0u, b = (uint8_t)(m - 1u); a < b; a++, b--)
    {
      char t = k[a];
      k[a]   = k[b];
      k[b]   = t;
    }

    (void)strcpy(tmp, "LIM ");
    tmp[4] = Stepper_LimitNearActive() ? '1' : '0';
    tmp[5] = '/';
    tmp[6] = Stepper_LimitFarActive() ? '1' : '0';
    (void)strcpy(&tmp[7], "   K");
    (void)strcpy(&tmp[11], k);

    (void)strcpy(lines[3], tmp);
  }
}

static void ui_task(void)
{
  static uint32_t last = 0u;
  static char     prev[4][17];
  char            cur[4][17];
  uint8_t         i;
  bool            dirty = false;

  if ((HAL_GetTick() - last) < 100u)
  {
    return;
  }
  last = HAL_GetTick();

  ui_build(cur);

  for (i = 0u; i < 4u; i++)
  {
    if (memcmp(cur[i], prev[i], 17u) != 0)
    {
      OLED_ShowLine((uint8_t)(i + 1u), cur[i]);
      (void)memcpy(prev[i], cur[i], 17u);
      dirty = true;
    }
  }

  if (dirty)
  {
    OLED_Refresh();
  }
}

/* -------------------------------------------------------------------------- */
/* LED                                                                        */
/* -------------------------------------------------------------------------- */
static void led_task(void)
{
  static uint32_t last = 0u;
  static uint8_t  on = 0u;
  uint32_t        now = HAL_GetTick();

  switch (s_state)
  {
    case APP_FAULT:
      if ((now - last) >= 100u)
      {
        last = now;
        on = (uint8_t)!on;
        HAL_GPIO_WritePin(LED_GPIO, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
      break;

    case APP_MOVING:
    case APP_HOMING:
      HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_SET);
      break;

    default:
      if ((now - last) >= 500u)
      {
        last = now;
        on = (uint8_t)!on;
        HAL_GPIO_WritePin(LED_GPIO, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
      break;
  }
}
