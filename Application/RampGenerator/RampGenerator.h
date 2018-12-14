/********************************************************************************
* FileName: RampGenerator.h
* Description:  
* This header file contains the decleration of all the attributes and 
* services for RampGenerator.c file. It implements ramp generator for 
* speed and current mode of operation
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
#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H
#include "./Common/Typedefs/Typedefs.h"
#include "./Common/UserDefinition/Userdef.h"


#define lockApply  (PORTCbits.RC9 = 0)
#define lockRelease (PORTCbits.RC9 = 1)


//PWM coasting time in ms
#define PWM_COASTING_TIME   200      //50ms

typedef enum rampState
{
    RAMP_START,
    RAMP_RESTART,
    RAMP_RUNNING,
    RAMP_STOP,
    RAMP_PWM_COASTING,
    RAMP_STATE_END
}rampState_en;

/* flags used for the application */
typedef struct StatusFlags
{
	unsigned motorRunning	:1;  /* This bit is 1 if motor running */
	unsigned speedOpenLoop  :1;  /* This bit is 1 if motor is running in open loop */
    unsigned StartStop      :1;  /* Start/Stop command for motor from DMCI */
    unsigned speedControl   :1;  /* Set to operate in speed mode */
    unsigned currentControl :1;  /* Set to operate in current mode */
    unsigned currOpenLoop	:1;  /* This bit is 1 if motor is running in open loop */
    unsigned exstFanOn      :1;   //indicates fan ON status
    unsigned MotorType      :1;
    unsigned RunDirection   :1;
	unsigned unused			:7;
}StatusFlags_t;

typedef struct _rampStatusFlags
{
	unsigned rampGenRunning	:1;
    unsigned rampDcInjectionOn :1;
    unsigned rampBrakeOn :1;
    unsigned shutterOperationStart :1;
    unsigned shutterOperationComplete :1;
    unsigned rampSpeedControlRequired:1;
    unsigned rampCurrentControlRequired:1;
    unsigned rampLockRelDmci:1;
    unsigned rampLockAppDmci:1;
    unsigned safetySensorTriggered:1;
    unsigned rampOpenInProgress:1;
    unsigned rampCloseInProgress:1;
    unsigned rampMaintainHoldingDuty:1;
    unsigned rampDriftCalculated:1;
    unsigned mechLockRelFlag:1;
	unsigned saveParamToEeprom:1;
}rampStatusFlags_t;

EXTERN StatusFlags_t flags;         /* Application status flag */
EXTERN rampStatusFlags_t rampStatusFlags;

/* This function charges bootstrap capacitor for quick start */
VOID chargeBootstraps(VOID);

/* This function starts all services required to run motor */
VOID startMotor(VOID);

/* This function stops all services required to run motor */
VOID stopMotor(VOID);
VOID forceStopShutter(VOID);

VOID overcurrentfaultTriggered(BOOL);
VOID checkPwmCoastingRequired(VOID);

VOID startMotorCW(VOID);
VOID startMotorCCW(VOID);


#endif /* RAMP_GENERATOR_H */
