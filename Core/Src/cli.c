/**
  ******************************************************************************
  * @file    cli.c
  * @brief   串口命令行。USART2 115200-8N1，DMA 循环收，IDLE 断帧，
  *          主循环里逐行解析，不阻塞运动。
  *
  *  指令（回车结束，大小写不敏感）：
  *    A<deg>  绝对角度定位      A90 / A-45.5
  *    R<deg>  相对角度          R10 / R-10
  *    V<rpm>  速度 0.1~120      V6
  *    + / -   连续点动          S 停止
  *    G       回零（找 PB0 近端限位）
  *    Z       当前位置清零
  *    S       停止
  *    C       清除报警
  *    K<spr>  每圈步数          K3200
  *    ?       状态    H  帮助
  ******************************************************************************
  */

#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "main.h"
#include "cli.h"
#include "app.h"
#include "stepper.h"

/* --------------------------------------------------------------------------
 * 控制台串口：USART3（PB10 = TX，PB11 = RX）
 * 说明：原来的 USART2(PA2/PA3) 上 PA2 实测 0V（引脚损坏），
 *       所以控制台整体搬到 USART3，OLED 仍在 PA8/PA9 不受影响。
 *       初始化写在这里（用户文件），CubeMX 重新生成不会覆盖。
 * ------------------------------------------------------------------------ */
static UART_HandleTypeDef s_uart;

#define RX_RING_LEN     256u
#define LINE_MAX_LEN    32u

static volatile uint8_t  rx_ring[RX_RING_LEN];
static volatile uint16_t rx_head = 0u;
static volatile uint16_t rx_tail = 0u;
static volatile uint8_t  rx_overflow = 0u;

static char    line_buf[LINE_MAX_LEN];
static uint8_t line_len = 0u;
static volatile uint32_t rx_byte_count  = 0u;   /* 上电以来收到的字节数，供 OLED 诊断 */
static volatile uint32_t tx_byte_count  = 0u;   /* HAL 成功发出的字节数 */
static volatile uint32_t tx_error_count = 0u;   /* HAL 发送失败次数 */

/* -------------------------------------------------------------------------- */
/* 格式化                                                                     */
/* -------------------------------------------------------------------------- */
void Fmt_Int(char *out, int32_t v)
{
  char    tmp[12];
  uint8_t n = 0u;
  uint8_t i;
  bool    neg = (v < 0);
  uint32_t a;

  if (neg)
  {
    a = (uint32_t)(-(v + 1)) + 1u;   /* 处理 INT32_MIN */
  }
  else
  {
    a = (uint32_t)v;
  }

  do
  {
    tmp[n++] = (char)('0' + (a % 10u));
    a /= 10u;
  } while ((a != 0u) && (n < 10u));

  if (neg)
  {
    *out++ = '-';
  }
  for (i = n; i > 0u; i--)
  {
    *out++ = tmp[i - 1u];
  }
  *out = '\0';
}

/**
  * @brief  定点一位小数，至少 3 位整数，例如 125 -> "+012.5"
  */
void Fmt_Fix1(char *out, int32_t v10)
{
  char     tmp[12];
  uint8_t  n = 0u;
  uint8_t  i;
  bool     neg = (v10 < 0);
  uint32_t a;
  uint32_t ip;
  uint32_t fp;

  if (neg)
  {
    a = (uint32_t)(-(v10 + 1)) + 1u;
  }
  else
  {
    a = (uint32_t)v10;
  }

  ip = a / 10u;
  fp = a % 10u;

  do
  {
    tmp[n++] = (char)('0' + (ip % 10u));
    ip /= 10u;
  } while ((ip != 0u) && (n < 9u));

  while (n < 3u)
  {
    tmp[n++] = '0';
  }

  if (neg)
  {
    *out++ = '-';
  }
  for (i = n; i > 0u; i--)
  {
    *out++ = tmp[i - 1u];
  }
  *out++ = '.';
  *out++ = (char)('0' + fp);
  *out = '\0';
}

/* -------------------------------------------------------------------------- */
/* 发送                                                                       */
/* -------------------------------------------------------------------------- */
void CLI_Print(const char *s)
{
  size_t len = strlen(s);
  if (len == 0u)
  {
    return;
  }

  {
    HAL_StatusTypeDef st = HAL_UART_Transmit(&s_uart, (uint8_t *)s, (uint16_t)len, 200u);

    if (st == HAL_OK)
    {
      tx_byte_count += (uint32_t)len;
    }
    else
    {
      tx_error_count++;
    }
  }
}

uint32_t CLI_GetTxCount(void)
{
  return tx_byte_count;
}

uint32_t CLI_GetTxErrors(void)
{
  return tx_error_count;
}

/* -------------------------------------------------------------------------- */
/* 接收（USART3 + RXNE 中断；不用 DMA，逻辑更简单）                            */
/* -------------------------------------------------------------------------- */
static void rx_push(uint8_t byte)
{
  uint16_t next = (uint16_t)((rx_head + 1u) % RX_RING_LEN);

  rx_byte_count++;

  if (next == rx_tail)
  {
    rx_overflow = 1u;      /* 队列满，丢弃新字节 */
  }
  else
  {
    rx_ring[rx_head] = byte;
    rx_head = next;
  }
}

void CLI_Init(void)
{
  GPIO_InitTypeDef g = {0};

  rx_head     = 0u;
  rx_tail     = 0u;
  rx_overflow = 0u;
  line_len    = 0u;

  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PB10 = USART3_TX */
  g.Pin   = GPIO_PIN_10;
  g.Mode  = GPIO_MODE_AF_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &g);

  /* PB11 = USART3_RX，上拉：线没接时保持空闲高，不会乱收 */
  g.Pin  = GPIO_PIN_11;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &g);

  s_uart.Instance           = USART3;
  s_uart.Init.BaudRate      = 115200u;
  s_uart.Init.WordLength    = UART_WORDLENGTH_8B;
  s_uart.Init.StopBits      = UART_STOPBITS_1;
  s_uart.Init.Parity        = UART_PARITY_NONE;
  s_uart.Init.Mode          = UART_MODE_TX_RX;
  s_uart.Init.HwFlowCtl     = UART_HWCONTROL_NONE;
  s_uart.Init.OverSampling  = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&s_uart) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_UART_ENABLE_IT(&s_uart, UART_IT_RXNE);

  HAL_NVIC_SetPriority(USART3_IRQn, 1u, 0u);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
  * @brief  USART3 中断：收到一个字节就放进环形队列，解析交给主循环
  */
void USART3_IRQHandler(void)
{
  uint32_t sr = USART3->SR;

  if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u)
  {
    uint8_t byte = (uint8_t)USART3->DR;   /* 读 SR 再读 DR：清 RXNE 和错误标志 */

    if ((sr & USART_SR_RXNE) != 0u)
    {
      rx_push(byte);
    }
  }
}

uint32_t CLI_GetRxCount(void)
{
  return rx_byte_count;
}

/* -------------------------------------------------------------------------- */
/* 指令解析                                                                   */
/* -------------------------------------------------------------------------- */
static bool parse_fix1(const char *s, int32_t *out)
{
  bool     neg = false;
  uint32_t ip = 0u;
  uint32_t fp = 0u;
  uint8_t  nd = 0u;

  while ((*s == ' ') || (*s == '=') || (*s == ':'))
  {
    s++;
  }
  if ((*s == '+') || (*s == '-'))
  {
    neg = (*s == '-');
    s++;
  }
  if ((*s < '0') || (*s > '9'))
  {
    return false;
  }

  while ((*s >= '0') && (*s <= '9'))
  {
    if (ip < 100000u)
    {
      ip = (ip * 10u) + (uint32_t)(*s - '0');
    }
    s++;
  }

  if (*s == '.')
  {
    s++;
    while ((*s >= '0') && (*s <= '9'))
    {
      if (nd < 1u)
      {
        fp = (fp * 10u) + (uint32_t)(*s - '0');
        nd++;
      }
      s++;
    }
  }

  while (*s == ' ')
  {
    s++;
  }
  if (*s != '\0')
  {
    return false;
  }

  while (nd < 1u)
  {
    fp *= 10u;
    nd++;
  }

  {
    int32_t value = (int32_t)((ip * 10u) + fp);
    *out = neg ? -value : value;
  }
  return true;
}

static bool parse_u32(const char *s, uint32_t *out)
{
  uint32_t v = 0u;
  uint8_t  n = 0u;

  while ((*s == ' ') || (*s == '=') || (*s == ':'))
  {
    s++;
  }
  if ((*s < '0') || (*s > '9'))
  {
    return false;
  }
  while ((*s >= '0') && (*s <= '9'))
  {
    if (n < 9u)
    {
      v = (v * 10u) + (uint32_t)(*s - '0');
      n++;
    }
    s++;
  }
  while (*s == ' ')
  {
    s++;
  }
  if (*s != '\0')
  {
    return false;
  }
  *out = v;
  return true;
}

static void print_help(void)
{
  CLI_Print("\r\n"
            " A<deg>  goto angle     A90  A-45.5\r\n"
            " R<deg>  relative       R10  R-10\r\n"
            " V<rpm>  speed          V6\r\n"
            " + / -   jog            S stop\r\n"
            " G  home   Z  zero   C  clear fault\r\n"
            " K<spr>  steps/rev      K3200\r\n"
            " ?  status   H  help\r\n");
}

static void reply_ok(const char *tag, int32_t deg10)
{
  char num[16];
  char buf[32];

  Fmt_Fix1(num, deg10);
  (void)strcpy(buf, "OK ");
  (void)strcat(buf, tag);
  (void)strcat(buf, num);
  (void)strcat(buf, "\r\n");
  CLI_Print(buf);
}

static void dispatch(char *s)
{
  char        cmd;
  int32_t     v;
  uint32_t    u;
  const char *arg;
  char        num[16];

  while (*s == ' ')
  {
    s++;
  }
  if (*s == '\0')
  {
    return;
  }

  cmd = (char)toupper((unsigned char)s[0]);
  arg = s + 1;

  switch (cmd)
  {
    case 'H':
      print_help();
      break;

    case '?':
      App_PrintStatus();
      break;

    case 'S':
      App_Stop();
      CLI_Print("OK STOP\r\n");
      break;

    case 'G':
      App_Home();
      break;

    case 'Z':
      App_Zero();
      CLI_Print("OK ZERO\r\n");
      break;

    case 'C':
      App_ClearFault();
      CLI_Print("OK CLEAR\r\n");
      break;

    case '+':
      if (App_Jog(1))
      {
        CLI_Print("OK JOG+\r\n");
      }
      else
      {
        CLI_Print("ERR JOG\r\n");
      }
      break;

    case '-':
      if (App_Jog(-1))
      {
        CLI_Print("OK JOG-\r\n");
      }
      else
      {
        CLI_Print("ERR JOG\r\n");
      }
      break;

    case 'A':
      if (!parse_fix1(arg, &v))
      {
        CLI_Print("ERR NUM\r\n");
      }
      else if (!App_MoveAbsDeg10(v))
      {
        CLI_Print("ERR A\r\n");
      }
      else
      {
        reply_ok("A", v);
      }
      break;

    case 'R':
      if (!parse_fix1(arg, &v))
      {
        CLI_Print("ERR NUM\r\n");
      }
      else if (!App_MoveRelDeg10(v))
      {
        CLI_Print("ERR R\r\n");
      }
      else
      {
        reply_ok("R", v);
      }
      break;

    case 'V':
      if (!parse_fix1(arg, &v) || (v < (int32_t)STEPPER_RPM10_MIN) ||
          (v > (int32_t)STEPPER_RPM10_MAX))
      {
        CLI_Print("ERR V (0.1-120.0)\r\n");
      }
      else
      {
        (void)App_SetSpeedRpm10((uint32_t)v);
        Fmt_Fix1(num, v);
        CLI_Print("OK V=");
        CLI_Print(num);
        CLI_Print("rpm\r\n");
      }
      break;

    case 'K':
      if (!parse_u32(arg, &u) || (u < STEPPER_SPR_MIN) || (u > STEPPER_SPR_MAX))
      {
        CLI_Print("ERR K (200-12800)\r\n");
      }
      else
      {
        (void)App_SetSpr(u);
        Fmt_Int(num, (int32_t)u);
        CLI_Print("OK K=");
        CLI_Print(num);
        CLI_Print("\r\n");
      }
      break;

    default:
      CLI_Print("ERR? H=help\r\n");
      break;
  }
}

/* -------------------------------------------------------------------------- */
/* 主循环轮询                                                                 */
/* -------------------------------------------------------------------------- */
/**
  * @brief  无参数的“单键指令”。串口助手不勾“末尾加回车换行”时也能直接用，
  *         例如只发一个 ? 就出状态。
  */
static bool is_instant_cmd(char c)
{
  switch ((char)toupper((unsigned char)c))
  {
    case '?':
    case 'H':
    case 'S':
    case 'G':
    case 'Z':
    case 'C':
    case '+':
    case '-':
      return true;
    default:
      return false;
  }
}

void CLI_Poll(void)
{
  while (rx_tail != rx_head)
  {
    char c = (char)rx_ring[rx_tail];

    rx_tail = (uint16_t)((rx_tail + 1u) % RX_RING_LEN);

    if ((c == '\r') || (c == '\n'))
    {
      if (line_len > 0u)
      {
        line_buf[line_len] = '\0';
        line_len = 0u;
        dispatch(line_buf);
      }
    }
    else if ((c == 0x08) || (c == 0x7F))
    {
      if (line_len > 0u)
      {
        line_len--;
      }
    }
    else if ((c >= 0x20) && (c < 0x7F))
    {
      if (line_len < (LINE_MAX_LEN - 1u))
      {
        line_buf[line_len++] = c;
        line_buf[line_len]   = '\0';

        /* 单键指令：收到一个字符就执行，不用等回车 */
        if ((line_len == 1u) && is_instant_cmd(c))
        {
          line_len = 0u;
          dispatch(line_buf);
        }
      }
      else
      {
        line_len = 0u;      /* 超长直接丢弃 */
      }
    }
    else
    {
      /* 控制字符忽略 */
    }
  }

  if (rx_overflow != 0u)
  {
    rx_overflow = 0u;
    CLI_Print("!RX OVF\r\n");
  }
}
