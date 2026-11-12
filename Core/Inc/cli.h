/**
  ******************************************************************************
  * @file    cli.h
  * @brief   USART2 指令解析：DMA 循环接收 + IDLE 断帧，逐行解析
  ******************************************************************************
  */

#ifndef __CLI_H
#define __CLI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void CLI_Init(void);
void CLI_Poll(void);
void CLI_Print(const char *s);
uint32_t CLI_GetRxCount(void);   /* 诊断用：累计收到的字节数 */
uint32_t CLI_GetTxCount(void);   /* 诊断用：HAL 成功发出的字节数 */
uint32_t CLI_GetTxErrors(void);  /* 诊断用：发送失败次数 */

/* 数值格式化（无 printf，省 Flash） */
void Fmt_Fix1(char *out, int32_t v10);   /* 12.5 -> "+012.5" */
void Fmt_Int(char *out, int32_t v);

#ifdef __cplusplus
}
#endif

#endif /* __CLI_H */
