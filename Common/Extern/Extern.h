/********************************************************************************
* FileName: Extern.h
* Description:  
* This header file declares common variables used by different modules.
*********************************************************************************/

/****************************************************************************
 * Copyright 2014 Bunka Shutters.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright
 * protection.
*****************************************************************************/

/****************************************************************************
 *  Modification History
 *  
 *  Date                  Name          Comments 
 *  09/04/2014            iGate          Initial Creation                                                               
*****************************************************************************/
#include "./Common/Typedefs/Typedefs.h"

EXTERN SHORT refSpeed;
EXTERN SHORT measuredSpeed;
EXTERN SHORT measuredSpeed_bak;
EXTERN DWORD period;
EXTERN SHORT controlOutput;
EXTERN SHORT ctrlOpPercent;
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
EXTERN SHORT hallCounts;
EXTERN SHORT refiTotalCurrent;
EXTERN DWORD iTotalADCCntAcc;
EXTERN WORD iTotalADCCntAccSmpls;
EXTERN BOOL calcTotalCurrentFlag;
EXTERN SHORT rampCurrentPosition;
EXTERN SHORT rampCurrentSpeed;
EXTERN SHORT rampCurrentState;

EXTERN BOOL emergencySensorTrigrd; 	
EXTERN BOOL microSwSensorTrigrd; 	
EXTERN BOOL airSwitchTrigrd; 		
EXTERN BOOL wrapAroundSensor; 		
EXTERN BOOL photoElecObsSensTrigrd; 
EXTERN BOOL tempSensTrigrd; 
EXTERN BOOL fourPtLmtSwtchDetected;
EXTERN BOOL originSensorDetected;       
EXTERN BOOL powerFailSensorDetected;
EXTERN BOOL updateSenStsCmd;
EXTERN WORD txResetCount;
EXTERN SHORT phaseInc;
EXTERN SHORT currentLimitClamp;
EXTERN WORD feedbackCurrent;
EXTERN SHORT outputDecRate;
EXTERN BOOL pwmCostingReq;
EXTERN WORD phaseValue;
extern WORD systemTick;
//	Added on 3 Feb 2015 to implement user control on power on calibration
EXTERN BYTE powerOnCalibration;
// Added to overcome installation issue (A100) - RN- NOV 2015
EXTERN BYTE gucInstallationInitiated;
EXTERN SHORT currShutterType;
//	Added this flag to handle issue related to stop action (stopping was not smooth) when stop key is pressed
extern BYTE gui8StopKeyPressed;

//	Shutter false direction movement monitoring flags

#define DEBUG_SHUTTER_MOVEMENT_IN_WRONG_DIRECTION

// Added for displaying errors on display screen in case of false movement - RN - NOV 2015
//	Flag to monitor shutter fall
extern BYTE gucShutterFalseUpMovementCount;
extern BYTE gucShutterFalseDownMovementCount;

//	Added for implementation of power fail functionality on DC Bus for version 4 board- RN- NOV 2015
EXTERN BYTE gucPowerFailFlag;
EXTERN BYTE gucPowerRestoredFlag;
EXTERN BYTE gucInstallationCalledFrom;

#ifdef DEBUG_SHUTTER_MOVEMENT_IN_WRONG_DIRECTION
//	Debug count for false direction movement
extern BYTE gucTempFalseMovementCount;
#endif

EXTERN BYTE MotorType;
EXTERN WORD UBUS;
EXTERN WORD MotorRunCount;
EXTERN WORD MotorCycleCount;
EXTERN BYTE MotorRunInCycle;
EXTERN BYTE MotorDecActive;
EXTERN UINT8 CMDStatus;
EXTERN WORD MotorStopCount;
EXTERN UINT8 CurrentMotorType;
EXTERN UINT8 PreMotorType;
EXTERN UINT16 PhaseAdvance;
EXTERN UINT16 DutyCycle;
EXTERN UINT16 DutyCycleSet;
EXTERN UINT16 PhaseURAT[];

EXTERN BYTE uart_motor_stop;
EXTERN BOOL Motor_ERR_overcurrent_or_igbtOverTemp;