/****************************************Copyright (c)**************************************************
**                         Burnon
**---------------文件信息--------------------------------------------------------------------------------
**	文      件  	名:	PI.H
**  创  建  日  期: by CPX 2015
**
**  文  件  说  明:
**
********************************************************************************************************/
#ifndef _PInew_H_
#define _PInew_H_

#include "stdint.h"

typedef struct {
	int32_t  Ref;
	int32_t  Fdb;
	int32_t  Err;
	int32_t  Kp;
	int32_t  Kix;
	int32_t  Kc;
	int32_t  Up;
	int32_t  Ui;
	int32_t  OutPreSat;
	int32_t  OutMax;
	int32_t  OutMin;
	int32_t  Out;
	int32_t  SatErr;
	uint8_t  State;
}PIDCTRL32;

extern PIDCTRL32 PI_SPL;
extern PIDCTRL32 PI_IDL;
extern PIDCTRL32 PI_IPL;

extern PIDCTRL32 PI_SPR;
extern PIDCTRL32 PI_IDR;
extern PIDCTRL32 PI_IPR;

extern int Uart_KPL_Speed_PI;
extern int Uart_KIL_Speed_PI;
extern int Uart_KDL_Speed_PI;
extern int Uart_KPL_Ibus_PI;
extern int Uart_KIL_Ibus_PI;
extern int Uart_KDL_Ibus_PI;



void PICal(PIDCTRL32 *v);
void PIRESet(PIDCTRL32 *v);
void PID_init(void);

#endif