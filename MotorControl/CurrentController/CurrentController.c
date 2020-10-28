/*********************************************************************************
* FileName: CurrentController.c
* Description:
* This source file contains the definition of all the functions for CurrentController.
* It implements all the functions required by current controller.
**********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
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
#include "./Drivers/GPIO/GPIO.h"

#define CURRENT_FILTER_CONST 70//100//500//1000 

#define MAX_ADC_VOLTAGE     3300

#define MAX_ADC_COUNT       1024 /* ADC configured in 10 bit mode */

#define CALCULATE_ADC_OFFSET_CNT    100  /* Number of samples taken for ADC offset calculation */

#define P_CURRENT_PI 13000
#define I_CURRENT_PI 900
#define C_CURRENT_PI 0x7FFF
#define MAX_CURRENT_PI    31128   //95% of max value ie 32767

#define ADC_CNT_TO_CURR_FACTOR      56  //MAX_TOTAL_CURRENT/ADC_CNT_AVAILABLE    //No Used
#define ADC_CNT_AVAILABLE           1024
#define ADC_MEASURABLE_CURRENT_VALUE    3300 //3.3A ie 3.3V = 3.3A

#define MAXIMUM_WORKING_VOLTAGE_ac   178  //??VAC
#define MAXIMUM_WORKING_VOLTAGE_DC   252  //MAXIMUM_WORKING_VOLTAGE_ac*1.414  //??VDC
#define MAXIMUM_WORKING_VOLTAGE      774  //(MAXIMUM_WORKING_VOLTAGE_DC*2.2k/(82k+82k+56k+2.2k))/3.3*1024

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
WORD phaseValue;

WORD UBUS = 0;
BOOL FLAG_read_UBUS;


/* This function is current PI controller */
VOID currentControl(VOID);

/* This function initializes all the variables used by current controller */
VOID initCurrentControllerVariables(VOID);

/* This function is used to measure total phase current of motor */
VOID measureTotalCurrent(VOID);

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
static unsigned int VBUS_value_Last;
	
    IFS1bits.AD2IF = 0;
	//	Capture DC Bus volatage
    
    VBUS_value_Last=UBUS;
    UBUS = ADC2BUF0;
    
    if((UBUS>MAXIMUM_WORKING_VOLTAGE)&&(VBUS_value_Last>MAXIMUM_WORKING_VOLTAGE))
    {
        Out_DBR_CTRL=1;
    }
    else Out_DBR_CTRL=0; 
	
	FLAG_read_UBUS=1;
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
        
    runCurrentLimitPI();
    
    //currentControl();
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

    //initPiNew(&currentPIparms,P_CURRENT_PI,I_CURRENT_PI,C_CURRENT_PI,currentLimitClamp,0,0);
    currentPIparms.qdSum = 0;
    currentPIparms.qKp = P_CURRENT_PI;
    currentPIparms.qKi = I_CURRENT_PI;
    currentPIparms.qKc = C_CURRENT_PI;
    currentPIparms.qOutMax = currentLimitClamp;
    currentPIparms.qOutMin = 0;
    currentPIparms.qOut = 0;    
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
    
        CalcPI(&currentPIparms);        
        if(flags.currentControl)
        {
            controlOutput = currentPIparms.qOut;
            speedPIparms.qdSum = currentPIparms.qdSum;
        }
    
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
	AD1CON1bits.ADON = 1;   //turn ON ADC module 
    delayMs(10);        /* wait for offset calculation */
    PTCONbits.PTEN = 0;	
    T2CONbits.TON = 0;
    TMR2 = 0;	 
	IEC0bits.T2IE = 0;	    
}


