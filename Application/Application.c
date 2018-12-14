/*********************************************************************************
* FileName: Application.c
* Description:
* This source file contains the definition of all the functions for the Application for Drive board.
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
 *  22/04/2014            iGate          Initial Creation                                                               
*****************************************************************************/
#include <p33Exxxx.h>
#include "Application.h"
#include "./Common/UserDefinition/Userdef.h"
#include "./Application/RampGenerator/RampGenerator.h"
#include "./Drivers/Timer/Timer.h"
#include "./Common/Extern/Extern.h"
#include "../Common/Delay/Delay.h"


WORD systemTick = 0;  
BOOL OvercurrentfaultTrigrd = FALSE; // indicating PWM fault due to overcurrent

BYTE MotorType;

UINT8 CMDStatus = 0;
BYTE MotorCountOver = 0;
WORD MotorStopCount;
UINT16 DutyCycleCnt = 0;

BYTE gui8StopKeyPressed = 0;
BYTE MotorRunInCycle = 0;


/******************************************************************************
 * getSystemTick
 *
 * This function gets system tick count 
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/  
UINT32 getSystemTick(VOID)
{
	return systemTick; 
}

/******************************************************************************
 * application
 *
 * This function implements the task loop for the Application
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
VOID application(VOID)
{
    if(MotorRunInCycle == 0)
    {
        CMDStatus = (UINT8) ((PORTDbits.RD8 << 1) | (PORTAbits.RA11));
        if((CMDStatus == 0x02) || (CMDStatus == 0x03))
        {
            Motor_ERR_overcurrent_or_igbtOverTemp=0;
        }
        
        if((CMDStatus == 0x00) && (!flags.motorRunning)&&(Motor_ERR_overcurrent_or_igbtOverTemp==0))
        {
            lockRelease;
            //delayMs(100); 
            flags.RunDirection = CCW;
            startMotorCCW();
        }
        else if((CMDStatus == 0x01) && (!flags.motorRunning)&&(Motor_ERR_overcurrent_or_igbtOverTemp==0))
        {
            lockRelease;
            //delayMs(100); 
            flags.RunDirection = CW;
            startMotorCW();
        }
        else if((CMDStatus == 0x01) && (flags.motorRunning) && (flags.RunDirection == CCW))
        {
            if(MotorDecActive==0)
            {
                MotorDecActive = 1;
                T9CONbits.TON = 1;
                MotorStopCount = 0;
            }
                if((MotorStopCount >= 40)||(measuredSpeed_bak<200))
                {
                    stopMotor(); 
                    delayMs(100);
                    lockApply;                    
                }
                else if(MotorStopCount >= 60)
                {
                    flags.RunDirection = CW;
                    startMotorCW();
                    //requiredDirection = CW;
                    //MotorDecActive = 0;
                    MotorStopCount = 0;
                    T9CONbits.TON = 0; 
                }            
            
        }
        else if((CMDStatus == 0x00) && (flags.motorRunning) && (flags.RunDirection == CW))
        {
            if(MotorDecActive==0)
            {
                MotorDecActive = 1;
                T9CONbits.TON = 1;
                MotorStopCount = 0;
            }
                if((MotorStopCount >= 40)||(measuredSpeed_bak<200))
                {
                    stopMotor(); 
                    delayMs(100);
                    lockApply;                    
                }
                else if(MotorStopCount >= 60)
                {
                    flags.RunDirection = CCW;
                    startMotorCCW();
                    //requiredDirection = CCW;
                    //MotorDecActive = 0;
                    MotorStopCount = 0;
                    T9CONbits.TON = 0; 
                }            
            
        }
        else if((CMDStatus == 0x02) && (flags.motorRunning))
        {
            if(MotorDecActive==0)
            {
                MotorDecActive = 1;
                T9CONbits.TON = 1;
                MotorStopCount = 0;
            }
                if((MotorStopCount >= 40)||(measuredSpeed_bak<200))
                {
                    stopMotor(); 
                    delayMs(100);
                    lockApply;                    
                }
            //stopMotor();

        }
        else if((CMDStatus == 0x03) && (flags.motorRunning))
        {
            if(MotorDecActive==0)
            {
                MotorDecActive = 1;
                T9CONbits.TON = 1;
                MotorStopCount = 0;
            }
                if((MotorStopCount >= 40)||(measuredSpeed_bak<200))
                {
                    stopMotor(); 
                    delayMs(100);
                    lockApply;                    
                }
            //stopMotor();
        }
    }

}

void __attribute__((interrupt, no_auto_psv)) _T9Interrupt (void)
{
    IFS3bits.T9IF = 0;
    MotorStopCount++;
    
}

