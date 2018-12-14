/*********************************************************************************
* FileName: RampGenerator.c
* Description:
* This source file contains the definition of all the functions for RampGenerator.
* It implements all the functions required by ramp generator.
**********************************************************************************/

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
#include <p33Exxxx.h>
#include "RampGenerator.h"
#include "./Common/UserDefinition/Userdef.h"
#include "./Common/Delay/Delay.h"
#include "./Common/Extern/Extern.h"
#include "./MotorControl/Algorithm/svm.h"
#include "./MotorControl/SpeedController/SpeedController.h"
#include "./MotorControl/CurrentController/CurrentController.h"
#include "./MotorControl/Braking/DCInjection.h"
#include "./Application/Application.h"
#include "./MotorControl/CurrentController/CurrentLimit.h"

#define CHARGE_BOOTSTRAP_CAP    1

StatusFlags_t flags;
///* Current mechanical motor direction of rotation Calculated in halls interrupts */
BYTE currentDirection;

BYTE requiredDirection;
SHORT currentRampProfileNo;
SHORT rampCurrentState;
rampStatusFlags_t rampStatusFlags;
BOOL pwmCostingReq = FALSE;


VOID pwmBufferControl(SHORT status)
{
    if(status == ENABLE)
    {
        enablePWMBuffer;
    }
    else
    {
        disablePWMBuffer;
    }
}

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
    IOCON1 = 0xC780;        
	IOCON2 = 0xC780;        
	IOCON3 = 0xC780;   
    PTCONbits.PTEN = 1;
    pwmBufferControl(ENABLE);
	delayMs(CHARGE_BOOTSTRAP_CAP); 
    pwmBufferControl(DISABLE);    
    IOCON1 = 0xF000;
    IOCON2 = 0xF000;
    IOCON3 = 0xF000;
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
    MotorRunCount = 0;
    //requiredDirection = CCW;
    //currentDirection = CCW;
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
    pwmBufferControl(ENABLE);
    AD1CON1bits.ADON = 1;   //turn ON ADC module 
	flags.motorRunning = 1;	/* Indicate that the motor is running */  
    
    flags.exstFanOn = 1; //set fan status flag
    fanON;               //Turn ON heat sink fan
    
}

VOID startMotorCW(VOID)
{
    flags.speedControl = 1;
#if (STARTUP_IN_CURRENT_MODE == 1)
    flags.currentControl = 1;
#else
    flags.currentControl = 0;
#endif
    MotorRunCount = 0;
    requiredDirection = CW;
    currentDirection = CW;
    
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
    pwmBufferControl(ENABLE);
    AD1CON1bits.ADON = 1;   //turn ON ADC module 
	flags.motorRunning = 1;	/* Indicate that the motor is running */  
    
    flags.exstFanOn = 1; //set fan status flag
    fanON;               //Turn ON heat sink fan
    
}

VOID startMotorCCW(VOID)
{
    flags.speedControl = 1;
#if (STARTUP_IN_CURRENT_MODE == 1)
    flags.currentControl = 1;
#else
    flags.currentControl = 0;
#endif
    MotorRunCount = 0;
    requiredDirection = CCW;
    currentDirection = CCW;
    MotorDecActive = 0;
    
    initCurrentLimitPI();
    intitSpeedController();     /* Initialize speed controller */
    intitCurrentController();   /* Initialize current controller */
    chargeBootstraps();
    
	TMR1 = 0;			/* Reset timer 1 for speed control */
    TMR2 = 0;			/* Reset timer 2 for current control */
	TMR3 = 0;			/* Reset timer 3 for speed measurement */
    TMR7 = 0;
    TMR8 = 0;
    
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
    pwmBufferControl(ENABLE);
    AD1CON1bits.ADON = 1;   //turn ON ADC module 
	flags.motorRunning = 1;	/* Indicate that the motor is running */  
    
    flags.exstFanOn = 1; //set fan status flag
    fanON;               //Turn ON heat sink fan
    
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
    pwmBufferControl(DISABLE);
    PTCONbits.PTEN = 0; 
    T7CONbits.TON = 0;
//	IEC0bits.T1IE = 0;
    IEC0bits.T2IE = 0;
    IEC0bits.T3IE = 0;
    IEC3bits.T7IE = 0;
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
    fanOFF;
    MotorRunCount = 0;
    MotorRunInCycle = 0;
}

//	Added on 20Feb2015 for IGBT over temperature fault
/******************************************************************************
 * _INT1Interrupt
 *
 * This function service the input change notification on port pin RB4
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
void __attribute__ ((interrupt, no_auto_psv)) _INT1Interrupt(void)
{
    IFS1bits.INT1IF = 0;
    //PORTAbits.RA7 = 0;
    
    if(PWMCON1bits.FLTSTAT)
		overcurrentfaultTriggered(TRUE);
#if 1
    else
        igbtOverTempSensorTriggered(TRUE);
#endif
}


//	Added on 20Feb2015 for IGBT over temperature fault
VOID igbtOverTempSensorTriggered(BOOL sts)
{
	//PORTAbits.RA7 = 0;
	
    //emergency stop sensor is normally open, it is triggered when closed
    if(sts)
    {
        //Sense emergency in all the profiles
            //if emergency switch is triggered the stop shutter immediately
            Motor_ERR_overcurrent_or_igbtOverTemp=1;
            forceStopShutter();
    }
	
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

VOID checkPwmCoastingRequired(VOID)
{
    if(rampCurrentState == RAMP_PWM_COASTING)
    {
        if(pwmCostingReq == TRUE)
        {
            if(currentLimitClamp > 0)
            {
                currentLimitClamp -= outputDecRate;
                if(currentLimitClamp < 0)
                    currentLimitClamp = 0;
            }
            else
            {
                currentLimitClamp = 0;
                pwmCostingReq = FALSE;
                forceStopShutter();
            }
        }        
    }
}


VOID forceStopShutter(VOID)
{
    stopMotor();
    lockApply;  
}

