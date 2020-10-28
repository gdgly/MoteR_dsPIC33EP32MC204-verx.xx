/********************************************************************************
* FileName: Extern.h
* Description:  
* This header file declares common variables used by different modules.
*********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#include "./Common/Typedefs/Typedefs.h"

//define IGBT type    
//#define IGBT_LowActive
#define IGBT_HighActive


EXTERN SHORT refSpeed;
EXTERN SHORT measuredSpeed;
EXTERN DWORD period;
EXTERN SHORT controlOutput;
EXTERN BYTE currentDirection;
EXTERN BYTE requiredDirection;
EXTERN WORD phase;
EXTERN WORD phaseCopy;
EXTERN SHORT phaseIncFlg;
EXTERN WORD hallValue;
EXTERN SHORT sector;
EXTERN CHAR sectorTable[];
EXTERN CONST SHORT phaseValues[];   
EXTERN SHORT phaseOffsetCW;
EXTERN SHORT phaseOffsetCCW;
EXTERN SHORT adcTrigger;
EXTERN SHORT iTotalADCCnt;
EXTERN SHORT iTotalOffset;
EXTERN DWORD iTotalInstFilter;
EXTERN WORD currentAverage;
EXTERN WORD measurediTotal;
EXTERN DWORD hallCounts;
EXTERN SHORT refiTotalCurrent;
EXTERN DWORD iTotalADCCntAcc;
EXTERN WORD iTotalADCCntAccSmpls;
EXTERN BOOL calcTotalCurrentFlag;
EXTERN SHORT rampCurrentPosition;
EXTERN SHORT rampCurrentSpeed;
EXTERN SHORT rampCurrentState;
	 			
EXTERN SHORT phaseInc;
EXTERN SHORT currentLimitClamp;
EXTERN WORD feedbackCurrent;
EXTERN SHORT outputDecRate;
EXTERN BOOL pwmCostingReq;
EXTERN WORD phaseValue;
extern WORD systemTick;



EXTERN WORD UBUS;
EXTERN BOOL FLAG_read_UBUS;
EXTERN BYTE MotorDecActive;
EXTERN UINT8 CMDStatus;
EXTERN WORD MotorStopCount;
EXTERN UINT16 PhaseAdvance;

EXTERN BOOL Motor_ERR_overcurrent_or_igbtOverTemp;


//*********************************this is new add********
extern unsigned int SET_SPEED;
extern BOOL FLAG_Motor_start;
extern UINT8 TIME_MotorForCurve;
extern BOOL FLAG_lockRelease;
