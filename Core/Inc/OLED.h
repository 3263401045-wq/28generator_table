/**
  ******************************************************************************
  * @file    OLED.h
  * @brief   SSD1306 128x64 软件 I2C 驱动（PA8 = SCL, PA9 = SDA，开漏）
  *
  *  与原始“江科大”例程的区别：
  *    1. 改用 HAL，去掉 stm32f10x.h / GPIO_WriteBit 等标准库接口；
  *    2. 引脚改到 PA8/PA9（CubeMX 已配成开漏输出）；
  *    3. 增加 1KB 显存，写字只改内存，OLED_Refresh() 才走 I2C，
  *       避免每个字符都开关总线导致主循环被拖慢。
  ******************************************************************************
  */

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String);
void OLED_ShowLine(uint8_t Line, const char *String);   /* 整行 16 字符，不足补空格 */

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
