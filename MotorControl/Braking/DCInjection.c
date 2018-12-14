/*********************************************************************************
* FileName: DCInection.c
* Description:
* This source file contains the definition of all the functions for DC Injection for motor holding.
* It implements all the functions required by DC Injection.
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
 *  09/05/2014            iGate          Initial Creation                                                               
*****************************************************************************/
#include <p33Exxxx.h>
#include "./MotorControl/Braking/DCInjection.h"
#include "./MotorControl/PIController/pi.h"
#include "./MotorControl/Algorithm/svm.h"
#include "./MotorControl/SpeedController/SpeedController.h"
#include "./Common/UserDefinition/Userdef.h"
#include "./Common/Extern/Extern.h"
#include "./Common/Delay/Delay.h"
#include "./Application/RampGenerator/RampGenerator.h"

/******************************************************************************
 * DCInjectionON
 *
 * This function controls motor hodling at one position .
 * 
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
VOID DCInjectionON(VOID)
{
    SHORT currentSector;
    
    currentSector = getCurrentSectorNo();    
	if (currentSector != -1)	/* If the sector is invalid don't do anything */
	{  
        calculatePhaseValue(currentSector);
        
        if(flags.motorRunning) 
        {
            IEC0bits.T1IE = 0;
            IEC0bits.T2IE = 0;
            IEC0bits.T3IE = 0;
            T1CONbits.TON = 0;
            T2CONbits.TON = 0;
            T3CONbits.TON = 0;
//	No need to disable hall interrupts
//	Changed to handle "offset at upper & lower limit"
#if 0
            IEC0bits.IC1IE = 0;	
            IEC0bits.IC2IE = 0;
            IEC2bits.IC3IE = 0;	           
#endif
        }
        else
        {
            chargeBootstraps();
                IFS5bits.PWM1IF = 0;
                IEC5bits.PWM1IE = 1;
                
            PTCONbits.PTEN = 1;
        }
    }
}

/******************************************************************************
 * DCInjectionOFF
 *
 * This function disables the motor hold and runs motor.
 * 
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
VOID DCInjectionOFF(VOID)
{    
    PTCONbits.PTEN = 0; 
}
