/********************************************************************************
* FileName: RampGenerator.h
* Description:  
* This header file contains the decleration of all the attributes and 
* services for RampGenerator.c file. It implements ramp generator for 
* speed and current mode of operation
*********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H
#include "./Common/Typedefs/Typedefs.h"
#include "./Common/UserDefinition/Userdef.h"


#define lockApply  {PORTCbits.RC9 = 0;FLAG_lockRelease=FALSE;}
#define lockRelease {PORTCbits.RC9 = 1;FLAG_lockRelease=TRUE;}
#define lockRelease_OUT PORTCbits.RC9


//PWM coasting time in ms
#define PWM_COASTING_TIME   200      //50ms

/* flags used for the application */
typedef struct StatusFlags
{
	unsigned motorRunning	:1;  /* This bit is 1 if motor running */
    unsigned StartStop      :1;  /* Start/Stop command for motor from DMCI */
    unsigned speedControl   :1;  /* Set to operate in speed mode */
    unsigned currentControl :1;  /* Set to operate in current mode */
    unsigned RunDirection   :1;
	unsigned unused			:11;
}StatusFlags_t;

EXTERN StatusFlags_t flags;         /* Application status flag */
/* This function charges bootstrap capacitor for quick start */
VOID chargeBootstraps(VOID);

/* This function starts all services required to run motor */
VOID startMotor(VOID);

/* This function stops all services required to run motor */
VOID stopMotor(VOID);
VOID forceStopShutter(VOID);

VOID overcurrentfaultTriggered(BOOL);


#endif /* RAMP_GENERATOR_H */
