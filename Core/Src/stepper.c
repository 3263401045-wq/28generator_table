/**
  ******************************************************************************
  * @file    stepper.c
  * @brief   TB6600 步进电机驱动：TIM3_CH1 输出脉冲，更新中断精确计数，
  *          PA12 输出方向，PB0/PB1 限位硬急停。
  *
  *  为什么用 PWM2 + 低极性：
  *    脉冲安排在“周期末尾”产生，计数器归零(更新事件)时输出即为无效电平(高)。
  *    这样在更新中断里停脉冲，刚好是上一拍脉冲已经走完，不会截断出一个
  *    宽度不足的毛刺，步数严格准确。
  ******************************************************************************
  */

#include "main.h"
#include "tim.h"
#include "stepper.h"

/* -------------------------------------------------------------------------- */
/* 引脚定义                                                                   */
/* -------------------------------------------------------------------------- */
#define DIR_GPIO            GPIOA
#define DIR_PIN             GPIO_PIN_12

#define LIMIT_GPIO          GPIOB
#define LIMIT_NEAR_PIN      GPIO_PIN_0
#define LIMIT_FAR_PIN       GPIO_PIN_1

/* -------------------------------------------------------------------------- */
/* 脉冲时序参数                                                               */
/* -------------------------------------------------------------------------- */
#define PULSE_WIDTH_US      10u         /* TB6600 最小 2.5us，取 10us 余量 */
#define MAX_STEP_FREQ_HZ    40000u      /* 28 电机够用，且留足中断余量 */
#define DIR_SETUP_US        20u         /* TB6600 要求方向建立 >= 5us */

/* -------------------------------------------------------------------------- */
/* 内部状态                                                                   */
/* -------------------------------------------------------------------------- */
static volatile int32_t s_pos          = 0;   /* 已走步数（脉冲计数）      */
static volatile int32_t s_left         = 0;   /* 剩余步数（连续模式不用）  */
static volatile int8_t  s_dir          = 1;   /* +1 / -1                   */
static volatile uint8_t s_state        = STEPPER_IDLE;
static volatile uint8_t s_continuous   = 0;   /* 1 = 连续点动              */
static volatile uint8_t s_limit_event  = 0;   /* 1 = 近端, 2 = 远端        */

static uint32_t s_spr   = STEPPER_DEFAULT_SPR;   /* 每圈步数 */
static uint32_t s_rpm10 = 60u;                   /* 默认 6.0 rpm */

static void pulse_stop(void);
static void apply_timing(void);

/* -------------------------------------------------------------------------- */
/* DWT 微秒延时                                                               */
/* -------------------------------------------------------------------------- */
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start  = DWT->CYCCNT;
  uint32_t cycles = us * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < cycles)
  {
  }
}

/* -------------------------------------------------------------------------- */
/* 脉冲输出控制                                                               */
/* -------------------------------------------------------------------------- */
/**
  * @brief  立即停止脉冲。计数器清零后 PWM2 输出回到无效电平（高），
  *         光耦不导通，电机保持当前位置。
  * @note   在中断与主循环中都会调用，寄存器操作本身是单条写，足够原子。
  */
static void pulse_stop(void)
{
  TIM3->CR1  &= ~TIM_CR1_CEN;
  TIM3->CNT   = 0u;
  TIM3->DIER &= ~TIM_DIER_UIE;
  TIM3->SR    = (uint16_t)~TIM_SR_UIF;
}

/**
  * @brief  根据当前转速/每圈步数计算 PSC/ARR/CCR。
  *         CCR 采用预装载，运动中改转速会在下一个周期整体生效，不会撕脉冲。
  */
static void apply_timing(void)
{
  uint64_t freq;
  uint32_t psc;
  uint32_t tick_hz;
  uint32_t arr;
  uint32_t pw;
  uint32_t ccr;

  freq = ((uint64_t)s_rpm10 * (uint64_t)s_spr) / 600u;   /* rpm10/10 * spr/60 */
  if (freq < 1u)
  {
    freq = 1u;
  }
  if (freq > MAX_STEP_FREQ_HZ)
  {
    freq = MAX_STEP_FREQ_HZ;
  }

  /* 分四档计数频率：高速档用 72MHz 保证分频分辨率，低速档保证 ARR 不溢出 */
  if (freq >= 20000u)
  {
    psc     = 0u;         /* 72MHz -> 13.9ns */
    tick_hz = 72000000u;
  }
  else if (freq >= 20u)
  {
    psc     = 71u;        /* 1MHz -> 1us */
    tick_hz = 1000000u;
  }
  else if (freq >= 2u)
  {
    psc     = 719u;       /* 100kHz -> 10us */
    tick_hz = 100000u;
  }
  else
  {
    psc     = 7199u;      /* 10kHz -> 100us，极低速用 */
    tick_hz = 10000u;
  }

  arr = tick_hz / (uint32_t)freq;
  if (arr < 4u)
  {
    arr = 4u;
  }
  if (arr > 65535u)
  {
    arr = 65535u;
  }
  arr -= 1u;

  /* 脉冲宽度换算成计数，至少 2 个计数，且不超过周期的一半 */
  pw = PULSE_WIDTH_US * (tick_hz / 1000000u);
  if (pw < 2u)
  {
    pw = 2u;
  }
  if (pw > (arr / 2u))
  {
    pw = arr / 2u;
  }
  if (pw < 2u)
  {
    pw = 2u;
  }

  /* PWM2 + 低极性：CNT >= CCR 时输出低，低电平长度 = arr - ccr + 1 */
  ccr = arr - pw + 1u;

  TIM3->PSC  = (uint16_t)psc;
  TIM3->ARR  = (uint16_t)arr;
  TIM3->CCR1 = (uint16_t)ccr;
}

/**
  * @brief  启动一串脉冲
  * @param  dir        方向 +1 / -1
  * @param  steps      步数（连续模式忽略）
  * @param  continuous 1 = 连续运转
  * @retval true 启动成功，false 被限位拒绝
  */
static bool start_train(int8_t dir, int32_t steps, bool continuous)
{
  if (dir == 0)
  {
    return false;
  }

  if (!continuous)
  {
    if ((dir < 0) && Stepper_LimitNearActive())
    {
      return false;
    }
    if ((dir > 0) && Stepper_LimitFarActive())
    {
      return false;
    }
  }

  pulse_stop();

  /* 方向必须先建立，再给脉冲 */
  HAL_GPIO_WritePin(DIR_GPIO, DIR_PIN, (dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  delay_us(DIR_SETUP_US);

  s_dir        = dir;
  s_left       = steps;
  s_continuous = continuous ? 1u : 0u;
  s_state      = continuous ? STEPPER_JOG : STEPPER_MOVING;

  TIM3->CNT = 0u;
  apply_timing();

  TIM3->EGR = TIM_EGR_UG;                      /* 立即装载 PSC/ARR/CCR */
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  TIM3->CR1 |= TIM_CR1_CEN;

  return true;
}

/* -------------------------------------------------------------------------- */
/* 更新中断：每来一个更新事件，说明上一拍脉冲已经完整发出                      */
/* -------------------------------------------------------------------------- */
void TIM3_IRQHandler(void)
{
  if ((TIM3->SR & TIM_SR_UIF) == 0u)
  {
    return;
  }
  TIM3->SR = (uint16_t)~TIM_SR_UIF;

  if (s_continuous != 0u)
  {
    s_pos += s_dir;
  }
  else if (s_left > 0)
  {
    s_pos += s_dir;
    s_left--;
    if (s_left == 0)
    {
      pulse_stop();
      s_state = STEPPER_IDLE;
    }
  }
}

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
void Stepper_Init(void)
{
  GPIO_InitTypeDef   io = {0};
  TIM_OC_InitTypeDef oc = {0};

  dwt_init();

  /* DIR 引脚（同时保证 CubeMX 重新生成后仍然可用） */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(DIR_GPIO, DIR_PIN, GPIO_PIN_RESET);
  io.Pin   = DIR_PIN;
  io.Mode  = GPIO_MODE_OUTPUT_PP;
  io.Pull  = GPIO_NOPULL;
  io.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DIR_GPIO, &io);

  /* 强制 PWM2 + 低极性，不依赖 tim.c 的默认配置 */
  oc.OCMode     = TIM_OCMODE_PWM2;
  oc.Pulse      = 500u;
  oc.OCPolarity = TIM_OCPOLARITY_LOW;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  (void)HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1);

  HAL_NVIC_SetPriority(TIM3_IRQn, 2u, 0u);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  s_pos         = 0;
  s_left        = 0;
  s_continuous  = 0u;
  s_limit_event = 0u;
  s_state       = STEPPER_IDLE;

  pulse_stop();
}

/* -------------------------------------------------------------------------- */
/* 参数设置                                                                   */
/* -------------------------------------------------------------------------- */
void Stepper_SetStepsPerRev(uint32_t spr)
{
  if (spr < STEPPER_SPR_MIN)
  {
    spr = STEPPER_SPR_MIN;
  }
  if (spr > STEPPER_SPR_MAX)
  {
    spr = STEPPER_SPR_MAX;
  }
  s_spr = spr;
  if (s_state != STEPPER_IDLE)
  {
    apply_timing();
  }
}

uint32_t Stepper_GetStepsPerRev(void)
{
  return s_spr;
}

void Stepper_SetSpeedRpm10(uint32_t rpm10)
{
  if (rpm10 < STEPPER_RPM10_MIN)
  {
    rpm10 = STEPPER_RPM10_MIN;
  }
  if (rpm10 > STEPPER_RPM10_MAX)
  {
    rpm10 = STEPPER_RPM10_MAX;
  }
  s_rpm10 = rpm10;

  /* 运动中改速度：预装载在下一个更新事件生效，相位连续 */
  if (s_state != STEPPER_IDLE)
  {
    apply_timing();
  }
}

uint32_t Stepper_GetSpeedRpm10(void)
{
  return s_rpm10;
}

/* -------------------------------------------------------------------------- */
/* 运动接口                                                                   */
/* -------------------------------------------------------------------------- */
bool Stepper_MoveToSteps(int32_t target_steps)
{
  int32_t delta;

  if (target_steps > STEPPER_POS_LIMIT)
  {
    target_steps = STEPPER_POS_LIMIT;
  }
  if (target_steps < -STEPPER_POS_LIMIT)
  {
    target_steps = -STEPPER_POS_LIMIT;
  }

  if (s_state != STEPPER_IDLE)
  {
    Stepper_Stop();
  }

  delta = target_steps - s_pos;
  if (delta == 0)
  {
    return true;
  }

  return start_train((delta > 0) ? 1 : -1,
                     (delta > 0) ? delta : -delta,
                     false);
}

bool Stepper_MoveSteps(int32_t steps)
{
  int32_t target;

  if (steps > STEPPER_POS_LIMIT)
  {
    steps = STEPPER_POS_LIMIT;
  }
  if (steps < -STEPPER_POS_LIMIT)
  {
    steps = -STEPPER_POS_LIMIT;
  }

  target = s_pos + steps;
  if (target > STEPPER_POS_LIMIT)
  {
    target = STEPPER_POS_LIMIT;
  }
  if (target < -STEPPER_POS_LIMIT)
  {
    target = -STEPPER_POS_LIMIT;
  }

  return Stepper_MoveToSteps(target);
}

bool Stepper_Jog(int8_t dir)
{
  if (s_state != STEPPER_IDLE)
  {
    Stepper_Stop();
  }
  return start_train(dir, 0, true);
}

void Stepper_Stop(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  pulse_stop();
  s_left       = 0;
  s_continuous = 0u;
  s_state      = STEPPER_IDLE;
  __set_PRIMASK(primask);
}

void Stepper_Zero(void)
{
  uint32_t primask;

  Stepper_Stop();

  primask = __get_PRIMASK();
  __disable_irq();
  s_pos = 0;
  __set_PRIMASK(primask);
}

/* -------------------------------------------------------------------------- */
/* 状态查询                                                                   */
/* -------------------------------------------------------------------------- */
bool Stepper_IsBusy(void)
{
  return (s_state != STEPPER_IDLE);
}

StepperState Stepper_GetState(void)
{
  return (StepperState)s_state;
}

int32_t Stepper_GetPosSteps(void)
{
  return s_pos;
}

int32_t Stepper_GetTargetSteps(void)
{
  if (s_continuous != 0u)
  {
    return s_pos;
  }
  return s_pos + (int32_t)s_dir * s_left;
}

/* -------------------------------------------------------------------------- */
/* 限位开关                                                                   */
/* -------------------------------------------------------------------------- */
bool Stepper_LimitNearActive(void)
{
  return (HAL_GPIO_ReadPin(LIMIT_GPIO, LIMIT_NEAR_PIN) == GPIO_PIN_RESET);
}

bool Stepper_LimitFarActive(void)
{
  return (HAL_GPIO_ReadPin(LIMIT_GPIO, LIMIT_FAR_PIN) == GPIO_PIN_RESET);
}

/**
  * @brief  EXTI 回调（HAL 弱函数重载），EXTI 优先级 0，立即停脉冲
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t which = 0u;

  if (GPIO_Pin == LIMIT_NEAR_PIN)
  {
    which = 1u;
  }
  else if (GPIO_Pin == LIMIT_FAR_PIN)
  {
    which = 2u;
  }
  else
  {
    return;
  }

  pulse_stop();
  s_left        = 0;
  s_continuous  = 0u;
  s_state       = STEPPER_IDLE;
  s_limit_event = which;
}

uint8_t Stepper_TakeLimitEvent(void)
{
  uint8_t  value;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  value         = s_limit_event;
  s_limit_event = 0u;
  __set_PRIMASK(primask);

  return value;
}

/* -------------------------------------------------------------------------- */
/* 角度换算（0.1 度，带四舍五入）                                              */
/* -------------------------------------------------------------------------- */
int32_t Stepper_StepsToDeg10(int32_t steps)
{
  int64_t v = (int64_t)steps * 3600;
  int64_t h = (int64_t)s_spr / 2;

  v = (v >= 0) ? (v + h) / (int64_t)s_spr : (v - h) / (int64_t)s_spr;
  return (int32_t)v;
}

int32_t Stepper_Deg10ToSteps(int32_t deg10)
{
  int64_t v = (int64_t)deg10 * (int64_t)s_spr;

  v = (v >= 0) ? (v + 1800) / 3600 : (v - 1800) / 3600;
  return (int32_t)v;
}
