/*********************************************************************************
* FileName: RampGenerator.c
* Description:
* This source file contains the definition of all the functions for RampGenerator.
* It implements all the functions required by ramp generator.
**********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#include <p33Exxxx.h>
#include "RampGenerator.h"
#include "./Common/UserDefinition/Userdef.h"
#include "./Common/Delay/Delay.h"
#include "./Common/Extern/Extern.h"
#include "./MotorControl/Algorithm/svm.h"
#include "./MotorControl/SpeedController/SpeedController.h"
#include "./MotorControl/CurrentController/CurrentController.h"
#include "./Application/Application.h"
#include "./MotorControl/CurrentController/CurrentLimit.h"

#define CHARGE_BOOTSTRAP_CAP    1

StatusFlags_t flags;
///* Current mechanical motor direction of rotation Calculated in halls interrupts */
BYTE currentDirection;

BYTE requiredDirection;
SHORT currentRampProfileNo;
rampStatusFlags_t rampStatusFlags;
BOOL pwmCostingReq = FALSE;
BOOL FLAG_Motor_start=FALSE;

BOOL FLAG_lockRelease=FALSE;

/******************************************************************************
 * chargeBootstraps
 *
 *  This function charges the bootstrap caps each time the motor is energized for the   
 *  first time after an undetermined amount of time. ChargeBootstraps subroutine turns
 *  ON the lower transistors for 10 ms to ensure voltage on these caps, and then it 
 *  transfers the control of the outputs to the PWM module.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/    
VOID chargeBootstraps(VOID)
{   
#ifdef IGBT_LowActive
    IOCON1 = 0xC780;
	IOCON2 = 0xC780;
	IOCON3 = 0xC780;
#endif
#ifdef IGBT_HighActive
    IOCON1 = 0xC740;
	IOCON2 = 0xC740;
	IOCON3 = 0xC740;
#endif 
    PTCONbits.PTEN = 1;
	delayMs(CHARGE_BOOTSTRAP_CAP);  
#ifdef IGBT_LowActive
    IOCON1 = 0xF000;
    IOCON2 = 0xF000;
    IOCON3 = 0xF000;
#endif
#ifdef IGBT_HighActive
    IOCON1 = 0xC000;
    IOCON2 = 0xC000;
    IOCON3 = 0xC000;
#endif
    PTCONbits.PTEN = 0;
    
    PDC1 = PHASE1 / 2;	// initialise as 0 volts
	PDC2 = PHASE2 / 2;	
	PDC3 = PHASE3 / 2;
}

/******************************************************************************
 * startMotor
 *
 * This function initializes speed controller and charges bootstrap capacitors.
 * It starts timer1 for speed PI controller, speed measurement and force 
 * commutation. it enables hall feedback and starts MCPWM to run the motor
 * 
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/  
VOID startMotor(VOID)
{
    flags.speedControl = 1;
#if (STARTUP_IN_CURRENT_MODE == 1)
    flags.currentControl = 1;
#else
    flags.currentControl = 0;
#endif
    if(flags.RunDirection == CCW)
    {
        requiredDirection = CCW;
        currentDirection = CCW;
    }
    else 
    {
        requiredDirection = CW;
        currentDirection = CW;        
    }
    MotorDecActive = 0;
    
    initCurrentLimitPI();
    intitSpeedController();     /* Initialize speed controller */
    intitCurrentController();   /* Initialize current controller */
    chargeBootstraps();
    
	TMR1 = 0;			/* Reset timer 1 for speed control */
    TMR2 = 0;			/* Reset timer 2 for current control */
	TMR3 = 0;			/* Reset timer 3 for speed measurement */
    
	T1CONbits.TON = 1;
    T2CONbits.TON = 1;
	T3CONbits.TON = 1;
    
	IFS0bits.T1IF = 0;		/* Clear timer 1 flag */
    IFS0bits.T2IF = 0;		/* Clear timer 2 flag */
    IFS0bits.T3IF = 0;		/* Clear timer 3 flag */
    
	IFS0bits.IC1IF = 0;		/* Clear interrupt flag */
	IFS0bits.IC2IF = 0;	
	IFS2bits.IC3IF = 0;

    
    IEC0bits.T1IE = 1;		/* Enable interrupts for timer 1 */
    IEC0bits.T2IE = 1;		/* Enable interrupts for timer 2 */
    IEC0bits.T3IE = 1;		/* Enable interrupts for timer 3 */
	IEC0bits.IC1IE = 1;		/* Enable interrupts on IC1 */
	IEC0bits.IC2IE = 1;		/* Enable interrupts on IC2 */
	IEC2bits.IC3IE = 1;

        IFS5bits.PWM1IF = 0;
        IEC5bits.PWM1IE = 1;
    
    PTCONbits.PTEN = 1;	    // start PWM  
    AD1CON1bits.ADON = 1;   //turn ON ADC module 
	flags.motorRunning = 1;	/* Indicate that the motor is running */  
    FLAG_Motor_start=TRUE;
    
}
/******************************************************************************
 * stopMotor
 *
 * This function stops timer1 for speed PI controller, speed measurement and force 
 * commutation. it disables hall feedback and stops MCPWM to stop the motor.
 * 
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
VOID stopMotor(VOID)
{
    PTCONbits.PTEN = 0; 
//	IEC0bits.T1IE = 0;
    IEC0bits.T2IE = 0;
    IEC0bits.T3IE = 0;
    IEC5bits.PWM1IE = 0;
//	T1CONbits.TON = 0;
    T2CONbits.TON = 0;
	T3CONbits.TON = 0;
//	No need to disable hall interrupts
//	Changed to handle "offset at upper & lower limit"
#if 0
	IEC0bits.IC1IE = 0;	
	IEC0bits.IC2IE = 0;
	IEC2bits.IC3IE = 0;	
#endif
    AD1CON1bits.ADON = 0;   //turn OFF ADC module 
	flags.motorRunning = 0;	/* Indicate that the motor has been stopped */
    MotorDecActive = 0;
}


VOID overcurrentfaultTriggered(BOOL sts)
{
    OvercurrentfaultTrigrd = sts;
    //emergency stop sensor is normally open, it is triggered when closed
    if(OvercurrentfaultTrigrd)
    {
        Motor_ERR_overcurrent_or_igbtOverTemp=1;
        //Sense emergency in all the profiles
            //if emergency switch is triggered the stop shutter immediately
            forceStopShutter();
    }
}


VOID forceStopShutter(VOID)
{
    stopMotor();
    lockApply;  
}

