/*********************************************************************************
* FileName: CurrentController.c
* Description:
* This source file contains the definition of all the functions for CurrentController.
* It implements all the functions required by current controller.
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
#include "CurrentController.h"
#include "CurrentLimit.h"
#include "./MotorControl/PIController/pi.h"
#include "./MotorControl/SpeedController/SpeedController.h"
#include "./Common/Delay/Delay.h"
#include "./Common/UserDefinition/Userdef.h"
#include "./Common/Extern/Extern.h"
#include "./Application/RampGenerator/RampGenerator.h"

#define CURRENT_FILTER_CONST 70//100//500//1000 

#define MAX_ADC_VOLTAGE     3300

#define MAX_ADC_COUNT       1024 /* ADC configured in 10 bit mode */

#define CALCULATE_ADC_OFFSET_CNT    100  /* Number of samples taken for ADC offset calculation */

#define P_CURRENT_PI 13000
#define I_CURRENT_PI 900
#define C_CURRENT_PI 0x7FFF
#define MAX_CURRENT_PI    31128   //95% of max value ie 32767

//#define ADC_CNT_TO_CURR_FACTOR      59//30000/512 = 58.59 = 59//MAX_TOTAL_CURRENT/512
//#define ADC_CNT_TO_CURR_FACTOR      83//42500/512 = 83.00 = 83//MAX_TOTAL_CURRENT/512
#define ADC_CNT_TO_CURR_FACTOR      56//28500/512 = 56.66 = 56.66//MAX_TOTAL_CURRENT/512
#define ADC_CNT_AVAILABLE           1024//512
#define ADC_MEASURABLE_CURRENT_VALUE    30000 //30.5A ie 3.3V = 30.5A

currCntrlFlg currControlFlag;

/* Variables used for offset calculation */
SHORT measureItotalOffsetCnt;
DWORD iTotalOffsetTot;
SHORT iTotalOffset;
SHORT adcCntToAmps;

/* Instantaneous total current ADC count */
SHORT iTotalADCCnt;

/* Instantaneous total current */
SHORT iTotalInst;

/* Filter used for current measurement */
WORD iTotalInstFilterConstant;
DWORD iTotalInstStateVar;
DWORD iTotalInstFilter;

/* total current for the PI */
WORD measurediTotal;  

/* Reference current for PI */
SHORT refiTotalCurrent;

DWORD iTotalADCCntAcc;
WORD iTotalADCCntAccSmpls;
BOOL calcTotalCurrentFlag;

/* PI configuration structure */
tPIParm currentPIparms;

WORD curriTotal;
WORD previTotal;
SHORT diffiTotal;
WORD phaseValue;

WORD UBUS = 0;

#define NO_LOAD_CURRENT_750W 300 //300 mA
#define NO_LOAD_CURRENT_1500W 400 //300 mA

#define CURRENT_DIFF_FOR_PHASE_INC_750W 275//225//250//300
#define CURRENT_DIFF_FOR_PHASE_INC_1500W 350

#define PHASE_INC_STEP  182
#define MAX_PHASE_COMP_750W  9646    //(PHASE_INC_STEP * 53)
#define MAX_PHASE_COMP_1500W  9828 
#define MIN_PHASE_COMP  0


/* This function is current PI controller */
VOID currentControl(VOID);

/* This function initializes all the variables used by current controller */
VOID initCurrentControllerVariables(VOID);

/* This function is used to measure total phase current of motor */
VOID measureTotalCurrent(VOID);

VOID checkPhaseCompensation(VOID);

/******************************************************************************
 * _AD1Interrupt
 *
 * The _AD1Interrupt loads the reference speed (RefSpeed) with
 * the respective value of the POT. The value will be a signed
 * fractional value, so it doesn't need any scaling.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/  
void __attribute__((interrupt, no_auto_psv)) _AD1Interrupt (void)
{
    IFS0bits.AD1IF = 0; 
    
    iTotalADCCnt = ADC1BUF0;   
    calculateTotalCurrent();
	//	Capture DC Bus current
	//uEEPDriveMotorCtrlBlock.stEEPDriveMotorCtrlBlock.PWMFreqMotorCtrl_A500 = ADC1BUF0;
}

//	Added for implementation of power fail functionality on DC Bus for version 4 board- RN- NOV 2015
void __attribute__((interrupt, no_auto_psv)) _AD2Interrupt (void)
{
	float lfDcBusVoltage = 0;
	//	Timer variables
	
    IFS1bits.AD2IF = 0;
	//	Capture DC Bus volatage
	lfDcBusVoltage = ADC2BUF0 / 2.25;	//	1V = 2.25 counts
    
    UBUS = (WORD) lfDcBusVoltage * 100;
	
	
}

//	Added for implementation of power fail functionality on DC Bus for version 4 board- RN- NOV 2015
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt (void)
{
	//static unsigned int lsui8Count = 0;
    IFS1bits.T5IF = 0;

}
VOID executePowerFailRoutine(VOID)
{
	
	forceStopShutter();
	
}

/******************************************************************************
 * _T2Interrupt
 *
 * The _T2Interrupt calculates total current of motor and runs current PI controller.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/  
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt (void)
{
	IFS0bits.T2IF = 0;
    
    checkPhaseCompensation();
    
    runCurrentLimitPI();
    
    //if( rampStatusFlags.rampCurrentControlRequired)
        currentControl();
}

VOID calculateTotalCurrent(VOID)
{
    if(!currControlFlag.offsetCalulated)
    {
        iTotalOffsetTot += iTotalADCCnt;
        if(++measureItotalOffsetCnt >= CALCULATE_ADC_OFFSET_CNT)
        {            
            iTotalOffset = __builtin_divud(iTotalOffsetTot,measureItotalOffsetCnt);     
            adcCntToAmps = __builtin_divud(ADC_MEASURABLE_CURRENT_VALUE, (ADC_CNT_AVAILABLE - iTotalOffset)); 
            currControlFlag.offsetCalulated = 1;
        }
    }
    else 
    {   
        iTotalInst = iTotalADCCnt;
        //check if we got some incorrect value then do not calculate current        
        if(iTotalInst > iTotalOffset)
        {
            iTotalInst -= iTotalOffset;
            filterTotalCurrent();
        }
    } 
}

VOID filterTotalCurrent(VOID)
{    
    iTotalInstStateVar+= ((iTotalInst - iTotalInstFilter)*(iTotalInstFilterConstant));
    iTotalInstFilter = iTotalInstStateVar>>15;    
    
    //measurediTotal = iTotalInstFilter * ADC_CNT_TO_CURR_FACTOR;
    measurediTotal = iTotalInstFilter * adcCntToAmps;    
    feedbackCurrent = measurediTotal;
}

VOID checkPhaseCompensation(VOID)
{
    WORD phaseIncAng = 0;
//    SHORT tmpDiff = 0;
    
    curriTotal = measurediTotal;
    //phase compensation should be applied when measured current is more than 300 mA 
    if(PreMotorType == MOTOR_750W)
    {
        if(curriTotal > NO_LOAD_CURRENT_750W)
        {
        //check difference between current and previous total current value
            diffiTotal = curriTotal - previTotal;
        //if difference is positive side then icnrement phase, if difference is negative side then 
        //decrement phse
            if(diffiTotal < 0)
            {
                if(diffiTotal <= (-CURRENT_DIFF_FOR_PHASE_INC_750W))
                {
                    diffiTotal = -diffiTotal;
                    phaseIncAng = __builtin_divud(diffiTotal,CURRENT_DIFF_FOR_PHASE_INC_750W);
                    if(phaseValue > MIN_PHASE_COMP)
                    {
                        phaseValue -= (PHASE_INC_STEP * phaseIncAng);
                    }
                    previTotal = curriTotal;
                }
            }
            else
            {   
                if(diffiTotal >= CURRENT_DIFF_FOR_PHASE_INC_750W)
                {
                    phaseIncAng = __builtin_divud(diffiTotal,CURRENT_DIFF_FOR_PHASE_INC_750W);
                    if(phaseValue < MAX_PHASE_COMP_750W)
                    {
                        phaseValue += (PHASE_INC_STEP * phaseIncAng);
                    }
                    previTotal = curriTotal;
                }
            }
        }
        else
        {
            phaseValue = MIN_PHASE_COMP * PHASE_INC_STEP;
        }
    }
    else if(PreMotorType == MOTOR_1500W)
    {
        if(curriTotal > NO_LOAD_CURRENT_1500W)
        {
            //check difference between current and previous total current value
            diffiTotal = curriTotal - previTotal;
            //if difference is positive side then icnrement phase, if difference is negative side then 
            //decrement phse
            if(diffiTotal < 0)
            {
                if(diffiTotal <= (-CURRENT_DIFF_FOR_PHASE_INC_1500W))
                {
                    diffiTotal = -diffiTotal;
                    phaseIncAng = __builtin_divud(diffiTotal,CURRENT_DIFF_FOR_PHASE_INC_1500W);
                    if(phaseValue > MIN_PHASE_COMP)
                    {
                        phaseValue -= (PHASE_INC_STEP * phaseIncAng);
                    }
                    previTotal = curriTotal;
                }
            }
            else
            {
                if(diffiTotal >= CURRENT_DIFF_FOR_PHASE_INC_1500W)
                {
                    phaseIncAng = __builtin_divud(diffiTotal,CURRENT_DIFF_FOR_PHASE_INC_1500W);
                    if(phaseValue < MAX_PHASE_COMP_1500W)
                    {
                        phaseValue += (PHASE_INC_STEP * phaseIncAng);
                    }
                    previTotal = curriTotal;
                }
            }
        }
        else
        {
            phaseValue = MIN_PHASE_COMP * PHASE_INC_STEP;
        }
    }
}

/******************************************************************************
 * intitCurrentController
 *
 * The intitCurrentController gives interface of speed controller to other modules. 
 * speed control mode.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
VOID intitCurrentController(VOID)
{
    initCurrentControllerVariables();    
}

/******************************************************************************
 * initCurrentControllerVariables
 *
 * The initCurrentControllerVariables inititializes all the variables used by 
 * current control mode.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
VOID initCurrentControllerVariables(VOID)
{
    iTotalADCCnt = 0;   
    iTotalInst = 1; 
    iTotalInstFilterConstant = CURRENT_FILTER_CONST;
    iTotalInstStateVar = ((DWORD)1 << 15);
    iTotalInstFilter = 1;                
    
    iTotalADCCntAcc = 0;
    iTotalADCCntAccSmpls = 0;
    calcTotalCurrentFlag = 0;
    measurediTotal = 0;

    curriTotal = 0;
    if(PreMotorType == MOTOR_750W)
    {
        previTotal = NO_LOAD_CURRENT_750W;
    }
    else if(PreMotorType == MOTOR_1500W)
    {
        previTotal = NO_LOAD_CURRENT_1500W;
    }
    
    diffiTotal = 0;
    phaseValue = MIN_PHASE_COMP * PHASE_INC_STEP;
    initPiNew(&currentPIparms,P_CURRENT_PI,I_CURRENT_PI,C_CURRENT_PI,currentLimitClamp,0,0);
}

/******************************************************************************
 * currentControl
 *
 * The currentControl implements current PI controller.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
VOID currentControl(VOID) 
{   
    currentPIparms.qInRef = refiTotalCurrent;
    currentPIparms.qInMeas = measurediTotal;  

    currentPIparms.qOutMax = currentLimitClamp;
    currentPIparms.qOutMin = 0;  
    
        calcPiNew(&currentPIparms);        
        if(flags.currentControl)
        {
            controlOutput = currentPIparms.qOut;
            speedPIparms.qdSum = currentPIparms.qdSum;
        }
    //}
    
    //calculate percentage duty
    ctrlOpPercent = __builtin_divud(((unsigned long)controlOutput*100),MAX_CURRENT_PI);
}

/******************************************************************************
 * measureADCOffset
 *
 * The measureADCOffset calculates ADC offset error
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
VOID measureADCOffset(VOID)
{
    initCurrentControllerVariables();    
    currControlFlag.offsetCalulated = 0;
    iTotalOffset = 0;
    measureItotalOffsetCnt = 0;
    iTotalOffsetTot = 0;
    chargeBootstraps();
    
    TMR2 = 0;			/* Reset timer 2 for current control */
    T2CONbits.TON = 1;
    IFS0bits.T2IF = 0;		/* Clear timer 2 flag */    
    IEC0bits.T2IE = 1;		/* Enable interrupts for timer 2 */       
    PDC1 = PHASE1 / 2;	/* Initialize as 0 voltage */
	PDC2 = PHASE2 / 2;
	PDC3 = PHASE3 / 2;	

        IFS5bits.PWM1IF = 0;
        IEC5bits.PWM1IE = 1;
        
    PTCONbits.PTEN = 1;	    // start PWM 
    //pwmBufferControl(ENABLE);
	AD1CON1bits.ADON = 1;   //turn ON ADC module 
    delayMs(10);        /* wait for offset calculation */
    //pwmBufferControl(DISABLE);
    PTCONbits.PTEN = 0;	
    T2CONbits.TON = 0;
    TMR2 = 0;	 
	IEC0bits.T2IE = 0;	    
}


