/**
  ******************************************************************************
  * @file    stepper.h
  * @brief   TB6600 pulse/direction stepper driver (TIM3_CH1 = PUL, PA12 = DIR)
  *
  *  硬件接线（与 TB6600 端子一一对应）：
  *    TB6600 PUL+  -> 3.3V         PUL-  -> PA6  (TIM3_CH1，低电平有效脉冲)
  *    TB6600 DIR+  -> 3.3V         DIR-  -> PA12 (推挽输出)
  *    TB6600 ENA+  -> 3.3V         ENA-  -> 悬空（驱动器常使能）
  *    TB6600 VCC/GND -> 12~24V 电源（必须与 STM32 共地）
  *
  *  脉冲时序：1MHz 计数，固定 10us 低电平脉冲，频率 = 转速 x 每圈步数 / 60。
  ******************************************************************************
  */

#ifndef __STEPPER_H
#define __STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 每圈步数 = 360 / 步距角 x 细分。1.8 度电机 1/16 细分 -> 3200 */
#define STEPPER_DEFAULT_SPR     3200u
#define STEPPER_SPR_MIN         200u
#define STEPPER_SPR_MAX         12800u

/* 转速设定范围，单位 0.1 rpm */
#define STEPPER_RPM10_MIN       1u
#define STEPPER_RPM10_MAX       1200u

/* 位置上限（步），防止 int32 溢出，约 30000 圈 @3200 */
#define STEPPER_POS_LIMIT       100000000L

typedef enum
{
  STEPPER_IDLE = 0,
  STEPPER_MOVING,
  STEPPER_JOG
} StepperState;

/* 初始化：DIR 引脚、PWM 模式、NVIC、DWT 延时 */
void     Stepper_Init(void);

/* 参数 */
void     Stepper_SetStepsPerRev(uint32_t spr);
uint32_t Stepper_GetStepsPerRev(void);
void     Stepper_SetSpeedRpm10(uint32_t rpm10);   /* 运动中调用会平滑生效 */
uint32_t Stepper_GetSpeedRpm10(void);

/* 运动：返回 false 表示被限位互锁拒绝 */
bool     Stepper_MoveSteps(int32_t steps);
bool     Stepper_MoveToSteps(int32_t target_steps);
bool     Stepper_Jog(int8_t dir);                 /* 连续转，dir = +1 / -1 */
void     Stepper_Stop(void);                      /* 立即停止脉冲 */
void     Stepper_Zero(void);                      /* 当前位置置零 */

/* 状态查询 */
bool          Stepper_IsBusy(void);
StepperState  Stepper_GetState(void);
int32_t       Stepper_GetPosSteps(void);
int32_t       Stepper_GetTargetSteps(void);

/* 限位开关（IN8 = PB0 负向/近端，IN9 = PB1 正向/远端，光耦低有效） */
bool     Stepper_LimitNearActive(void);
bool     Stepper_LimitFarActive(void);
uint8_t  Stepper_TakeLimitEvent(void);            /* 1 = 近端, 2 = 远端, 0 = 无 */

/* 角度换算，单位 0.1 度 */
int32_t  Stepper_StepsToDeg10(int32_t steps);
int32_t  Stepper_Deg10ToSteps(int32_t deg10);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_H */
