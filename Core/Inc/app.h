/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层：定位/回零/限位报警/OLED 界面
  ******************************************************************************
  */

#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum
{
  APP_READY = 0,
  APP_MOVING,
  APP_HOMING,
  APP_FAULT
} AppState;

void        App_Init(void);
void        App_Poll(void);

bool        App_MoveAbsDeg10(int32_t deg10);
bool        App_MoveRelDeg10(int32_t deg10);
bool        App_Jog(int8_t dir);
bool        App_SetSpeedRpm10(uint32_t rpm10);   /* 转台转速，0.1 rpm */
uint32_t    App_GetSpeedRpm10(void);             /* 转台转速，0.1 rpm */
bool        App_SetSpr(uint32_t spr);

void        App_Stop(void);
void        App_Zero(void);
void        App_Home(void);
void        App_ClearFault(void);

void        App_PrintStatus(void);
const char *App_StateStr(void);
AppState    App_GetState(void);
uint8_t     App_GetFault(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
