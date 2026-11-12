/**
  ******************************************************************************
  * @file    OLED.c
  * @brief   SSD1306 128x64 软件 I2C 驱动（HAL 版）
  *
  *  连线：OLED VCC -> 3.3V   GND -> GND   SCL -> PA8   SDA -> PA9
  *        模块一般自带 4.7k 上拉，PA8/PA9 已配成开漏 + 内部上拉。
  ******************************************************************************
  */

#include <string.h>

#include "main.h"
#include "OLED.h"
#include "OLED_Font.h"

#define OLED_I2C_ADDR   0x78u
#define OLED_WIDTH      128u
#define OLED_PAGES      8u

#define SCL_SET()       (GPIOA->BSRR = GPIO_BSRR_BS8)
#define SCL_CLR()       (GPIOA->BSRR = GPIO_BSRR_BR8)
#define SDA_SET()       (GPIOA->BSRR = GPIO_BSRR_BS9)
#define SDA_CLR()       (GPIOA->BSRR = GPIO_BSRR_BR9)

static uint8_t OLED_Buf[OLED_PAGES][OLED_WIDTH];
static uint8_t OLED_Dirty = 0u;      /* 位图：每 page 一位，只有脏页才走 I2C */

/* -------------------------------------------------------------------------- */
/* 软件 I2C                                                                   */
/* -------------------------------------------------------------------------- */
static void i2c_delay(void)
{
  for (volatile uint8_t i = 0u; i < 8u; i++)
  {
    __NOP();
  }
}

static void i2c_start(void)
{
  SDA_SET();
  i2c_delay();
  SCL_SET();
  i2c_delay();
  SDA_CLR();
  i2c_delay();
  SCL_CLR();
  i2c_delay();
}

static void i2c_stop(void)
{
  SDA_CLR();
  i2c_delay();
  SCL_SET();
  i2c_delay();
  SDA_SET();
  i2c_delay();
}

static void i2c_write(uint8_t byte)
{
  uint8_t i;

  for (i = 0u; i < 8u; i++)
  {
    if ((byte & 0x80u) != 0u)
    {
      SDA_SET();
    }
    else
    {
      SDA_CLR();
    }
    i2c_delay();
    SCL_SET();
    i2c_delay();
    SCL_CLR();
    i2c_delay();
    byte = (uint8_t)(byte << 1);
  }

  /* 第 9 个时钟，读应答但不处理 */
  SCL_SET();
  i2c_delay();
  SCL_CLR();
  i2c_delay();
}

static void oled_write_cmd(uint8_t cmd)
{
  i2c_start();
  i2c_write(OLED_I2C_ADDR);
  i2c_write(0x00u);
  i2c_write(cmd);
  i2c_stop();
}

/**
  * @brief  一次事务写完整个 page（128 字节），比逐字节开关总线快得多
  */
static void oled_write_page(uint8_t page, const uint8_t *data)
{
  uint8_t i;

  oled_write_cmd((uint8_t)(0xB0u + page));
  oled_write_cmd(0x00u);
  oled_write_cmd(0x10u);

  i2c_start();
  i2c_write(OLED_I2C_ADDR);
  i2c_write(0x40u);
  for (i = 0u; i < OLED_WIDTH; i++)
  {
    i2c_write(data[i]);
  }
  i2c_stop();
}

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
void OLED_Init(void)
{
  HAL_Delay(100u);              /* 上电等待模块内部复位 */

  oled_write_cmd(0xAEu);        /* 关显示 */
  oled_write_cmd(0xD5u);        /* 时钟分频 */
  oled_write_cmd(0x80u);
  oled_write_cmd(0xA8u);        /* 多路复用率 1/64 */
  oled_write_cmd(0x3Fu);
  oled_write_cmd(0xD3u);        /* 显示偏移 */
  oled_write_cmd(0x00u);
  oled_write_cmd(0x40u);        /* 起始行 */
  oled_write_cmd(0xA1u);        /* 左右方向 */
  oled_write_cmd(0xC8u);        /* 上下方向 */
  oled_write_cmd(0xDAu);        /* COM 引脚配置 */
  oled_write_cmd(0x12u);
  oled_write_cmd(0x81u);        /* 对比度 */
  oled_write_cmd(0xCFu);
  oled_write_cmd(0xD9u);        /* 预充电 */
  oled_write_cmd(0xF1u);
  oled_write_cmd(0xDBu);        /* VCOMH */
  oled_write_cmd(0x30u);
  oled_write_cmd(0xA4u);        /* 输出跟随 RAM */
  oled_write_cmd(0xA6u);        /* 正常显示 */
  oled_write_cmd(0x8Du);        /* 电荷泵开 */
  oled_write_cmd(0x14u);
  oled_write_cmd(0xAFu);        /* 开显示 */

  (void)memset(OLED_Buf, 0, sizeof(OLED_Buf));
  OLED_Dirty = 0xFFu;
  OLED_Refresh();
}

/* -------------------------------------------------------------------------- */
/* 显存操作                                                                   */
/* -------------------------------------------------------------------------- */
void OLED_Clear(void)
{
  (void)memset(OLED_Buf, 0, sizeof(OLED_Buf));
  OLED_Dirty = 0xFFu;
}

void OLED_Refresh(void)
{
  uint8_t page;

  for (page = 0u; page < OLED_PAGES; page++)
  {
    if ((OLED_Dirty & (uint8_t)(1u << page)) != 0u)
    {
      oled_write_page(page, OLED_Buf[page]);
      OLED_Dirty &= (uint8_t)~(1u << page);
    }
  }
}

/**
  * @brief  写一个 8x16 ASCII 字符到显存
  * @param  Line   1~4
  * @param  Column 1~16
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
  const uint8_t *glyph;
  uint8_t        x;
  uint8_t        i;
  uint8_t        idx;

  if ((Line < 1u) || (Line > 4u) || (Column < 1u) || (Column > 16u))
  {
    return;
  }
  if ((Char < ' ') || (Char > '~'))
  {
    Char = ' ';
  }

  idx   = (uint8_t)(Char - ' ');
  glyph = &OLED_F8x16[(uint16_t)idx * 16u];
  x     = (uint8_t)((Column - 1u) * 8u);

  for (i = 0u; i < 8u; i++)
  {
    OLED_Buf[(Line - 1u) * 2u][x + i]      = glyph[i];
    OLED_Buf[((Line - 1u) * 2u) + 1u][x + i] = glyph[i + 8u];
  }

  OLED_Dirty |= (uint8_t)((1u << ((Line - 1u) * 2u)) |
                          (1u << (((Line - 1u) * 2u) + 1u)));
}

void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String)
{
  while ((*String != '\0') && (Column <= 16u))
  {
    OLED_ShowChar(Line, Column, *String);
    String++;
    Column++;
  }
}

/**
  * @brief  整行显示 16 个字符，后面不足补空格（用于局部刷新）
  */
void OLED_ShowLine(uint8_t Line, const char *String)
{
  uint8_t col;

  for (col = 0u; col < 16u; col++)
  {
    char c = String[col];

    OLED_ShowChar(Line, (uint8_t)(col + 1u), (c == '\0') ? ' ' : c);
  }
}
