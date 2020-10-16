/*********************************************************************************
* FileName: SpeedController.c
* Description:
* This source file contains the definition of all the functions for SpeedController.
* It implements all the functions required by speed controller.
**********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#include <p33Exxxx.h>
#include "SpeedController.h"
#include "./MotorControl/PIController/pi.h"
#include "./MotorControl/Algorithm/svm.h"
#include "./Common/UserDefinition/Userdef.h"
#include "./Common/Extern/Extern.h"
#include "./Common/Delay/Delay.h"
#include "./Application/RampGenerator/RampGenerator.h"
#include "./Drivers/GPIO/GPIO.h"
#include "./Application/Application.h"

//	Macro to enable feature of motor cable fault - Dec 2015
//#define ENABLE_MOTOR_CABLE_FAULT

#define PERIOD_FILTER_CONST 2000//2500//2000 /* the smaller the value, the higher the filter
                                    //and the delay introduced */
/* In the sinewave generation algorithm we need an offset to be added to the */
/* pointer when energizing the motor */

#ifdef USE_PHASE_INC_AND_CORRECTION
#define PHASE_OFFSET_CW_750W  364//8500//2002     //1274 //measured offset is 1274*(360/65536) = 7 degrees.
#define PHASE_OFFSET_CCW_750W 10558//10740     //9828//9646//53 degrees
#else
#define PHASE_OFFSET_CW 10000 
#define PHASE_OFFSET_CCW 0
#endif
/*******************************************************************************/ 
/* These Phase values represent the base Phase value of the sinewave for each  */
/* one of the sectors (each sector is a translation of the hall effect sensors */
/* reading */
/****************************************************************************/
#define PHASE_ZERO 	0
#define PHASE_ONE	((PHASE_ZERO + 65536/6) % 65536)
#define PHASE_TWO	((PHASE_ONE + 65536/6) % 65536)
#define PHASE_THREE	((PHASE_TWO + 65536/6) % 65536)
#define PHASE_FOUR	((PHASE_THREE + 65536/6) % 65536)
#define PHASE_FIVE	((PHASE_FOUR + 65536/6) % 65536)

#define INVALID     -1
#define CNT_40MS    40      /* Used as a timeout with no hall effect sensors */
                            /* transitions and Forcing steps according to the */
                            /* actual position of the motor */

#define MS_500T 1000//300//500          /* after this time has elapsed, the motor is    */
                            /* consider stalled and it's stopped    */
       
// PI parameters        
#define P_SPEED_PI_MoteR Q15(0.85)   //prop
#define I_SPEED_PI_MoteR Q15(0.03)  //integ
#define C_SPEED_PI 0x7FFF                   //windup
#define MAX_SPEED_PI    31128   //95% of max value ie 32767

/* In the sinewave generation algorithm we need an offset to be added to the */
/* pointer when energizing the motor in CCW. This is done to compensate an   */
/* asymetry of the sinewave */
SHORT phaseOffsetCW =PHASE_OFFSET_CW_750W;
SHORT phaseOffsetCCW =PHASE_OFFSET_CCW_750W;


#define SPD_CAL_FOR_PHASEADVANCE    (int)((float)((((float)(measuredSpeed / 60) * NO_POLEPAIRS_750W) * 360) / 1000))
#define MAX_PH_ADV_DEG      1
#define MAX_PH_ADV 		(int)(((float)MAX_PH_ADV_DEG / 360.0) * 65536.0)
UINT16 PhaseAdvance;


/* Period filter for speed measurement */
DWORD periodFilter;
UINT16	cnt_motor_stop = 0;

/* Constants used for properly energizing the motor depending on the rotor's position */
CONST SHORT phaseValues[SECTOR_END] =
{PHASE_ZERO, PHASE_ONE, PHASE_TWO, PHASE_THREE, PHASE_FOUR, PHASE_FIVE}; 

/* PI configuration structure */
tPIParm speedPIparms;

/* This variable is incremented by the PWM interrupt in order to generate a proper sinewave. Its value */
/* is incremented by a value of PhaseInc, which represents the frequency of the generated sinewave */
WORD phase;
WORD phaseCopy;
SHORT phaseIncFlg;

/* This variable holds the hall sensor input readings */
WORD hallValue;

/* This variables holds present sector value, which is the rotor position */
SHORT sector;

/* This variable holds the last sector value. This is critical to filter slow slew rate on the Hall */
/* effect sensors hardware */
WORD lastSector;

/* This array translates the hall state value read from the digital I/O to the */
/* proper sector.  Hall values of 0 or 7 represent illegal values and therefore */
/* return -1. */


CHAR sectorTable[8]={-1,4,2,3,0,5,1,-1};
  
/* Variables containing the Period of half an electrical cycle, which is an */
/* interrupt each edge of one of the hall sensor input */
DWORD period;
DWORD phaseIncPerSec = 0;

/* Used as a temporal variable to perform a fractional divide operation in */
/* assembly */
SHORT measuredSpeed;  /* Actual speed for the PID */
SHORT measuredSpeed_bak;
BOOL Motor_ERR_overcurrent_or_igbtOverTemp;
SHORT refSpeed = 200;	    /* Desired speeds for the PID */ 

/* Output of PID controller, use its sign for required direction */
SHORT controlOutput;

/* Filter used for speed measurement */
WORD periodFilterConstant;
DWORD periodStateVar;	

//SHORT speedError;

//Observed hall counts for one hall sensor (IC2) is 155, 148, 151
DWORD hallCounts = 0;

/* Variable used by inbuilt division function */
UINT tmpQu = 0;
UINT tmpRe = 0;

SHORT hall2Triggered;
DWORD totalTimePeriod;

SHORT phaseInc;

WORD MotorCycleCount = 0;
BYTE MotorDecActive = 0;

/* This function is used to measure actual running speed of motor */
VOID measureActualSpeed(VOID);

/* This function is speed PI controller */
VOID speedControl(VOID);

/* This function initializes all the variables used by speed controller */
VOID initSpeedControllerVariables(VOID);


/******************************************************************************
 * _T1Interrupt
 *
 * The _T1Interrupt calculates actual speed of motor and runs speed PI controller.
 * It checks for motor stalled and calls force commutation to run motor. If the 
 * motor is stalled for more than 1 sec then stop all services to run the motor.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/  
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{   
    IFS0bits.T1IF = 0;

    TIME_MotorForCurve++;        
	measureActualSpeed();   
    if(++cnt_motor_stop>CNT_40MS)
		measuredSpeed = 0;
    
    speedControl(); 
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void)
{
    IFS0bits.T3IF = 0;
    totalTimePeriod += PR3;
}

SHORT getCurrentSectorNo(VOID)
{

    hallValue = HALLA_BIT + (HALLB_BIT << 1) + (HALLC_BIT << 2);      
	sector = sectorTable[hallValue];	//Get sector from table
   
    if(sector == INVALID)
    {
        sector = lastSector;       
    }  
    return(sector);
}
                
/******************************************************************************
 * _IC1Interrupt
 *
 * The _IC1Interrupt calculates the actual mechanical direction of rotation of 
 * the motor, and adjust the Phase variable depending on the sector the rotor is in.
 * The sector is validated in order to avoid any spurious interrupt due to a slow
 * slew rate on the halls inputs due to hardware filtering. For Phase adjustment in
 * CCW, an offset is added to compensate non-symetries in the sine table used.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt (void)
{
    SHORT currentSector;
	//	Added to handle "offset at upper & lower limit"
    
    IFS0bits.IC1IF = 0;
    
    currentSector = getCurrentSectorNo();
    
    /* This MUST be done for getting around the HW slow rate */
	//	Added check for invalid sector to handle "offset at upper & lower limit"
	if ((currentSector != lastSector) && (sector != INVALID))	
	{         
            //if ((currentSector == SECTOR_ZERO) || (currentSector == SECTOR_THREE))       
            if ((currentSector == sectorTable[2]) || (currentSector == sectorTable[5])) 
            {
                    currentDirection = CW;
                    if(M_Flags.flag_CW==0)hallCounts++;
                    else hallCounts--;  
            }
            else
            {
                    currentDirection = CCW;
                    if(M_Flags.flag_CW==0)hallCounts--;
                    else hallCounts++;
            }
            calculatePhaseValue(currentSector);
            lastSector = currentSector; /* Update last sector */
    }
}


/******************************************************************************
 * _IC2Interrupt
 *
 * The _IC2Interrupt calculates the actual period between hall effect sensor  
 * transitions and it calculates the actual mechanical direction of rotation of 
 * the motor, and adjust the Phase variable depending on the sector the rotor is in.
 * The sector is validated in order to avoid any spurious interrupt due to a slow
 * slew rate on the halls inputs due to hardware filtering. For Phase adjustment in
 * CCW, an offset is added to compensate non-symetries in the sine table used.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt (void)
{
	SHORT currentSector;
    
	
	//	Added to handle "offset at upper & lower limit"	
    
    IFS0bits.IC2IF = 0;
    
    currentSector = getCurrentSectorNo();
     
    /* This MUST be done for getting around the HW slow rate */
	//	Added check for invalid sector to handle "offset at upper & lower limit"
	if ((currentSector != lastSector) && (sector != INVALID))
	{          
        T3CONbits.TON = 0;
        totalTimePeriod += TMR3;
		TMR3 = 0;
		T3CONbits.TON = 1;
		hall2Triggered = 1;
                
		/* Motor current direction is computed based on sector */        
            //if ((currentSector == SECTOR_FIVE) || (currentSector == SECTOR_TWO))    
            if ((currentSector == sectorTable[3]) || (currentSector == sectorTable[4])) 
            {
                    currentDirection = CW;
                    if(M_Flags.flag_CW==0)hallCounts++;
                    else hallCounts--;  
            }
            else
            {
                    currentDirection = CCW;
                    if(M_Flags.flag_CW==0)hallCounts--;
                    else hallCounts++;
            }
            calculatePhaseValue(currentSector);
            lastSector = currentSector; /* Update last sector */
    }
}

/******************************************************************************
 * _IC3Interrupt
 *
 * The _IC3Interrupt calculates the actual mechanical direction of rotation of 
 * the motor, and adjust the Phase variable depending on the sector the rotor is in.
 * The sector is validated in order to avoid any spurious interrupt due to a slow
 * slew rate on the halls inputs due to hardware filtering. For Phase adjustment in
 * CCW, an offset is added to compensate non-symetries in the sine table used.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
void __attribute__((interrupt, no_auto_psv)) _IC3Interrupt (void)
{
    SHORT currentSector;
	
	//	Added to handle "offset at upper & lower limit"	
    
    IFS2bits.IC3IF = 0;
    
    currentSector = getCurrentSectorNo();    
     
    /* This MUST be done for getting around the HW slow rate */
	//	Added check for invalid sector to handle "offset at upper & lower limit"
	if ((currentSector != lastSector) && (sector != INVALID))
	{               
            //if ((currentSector == SECTOR_ONE) || (currentSector == SECTOR_FOUR))    
            if ((currentSector == sectorTable[1]) || (currentSector == sectorTable[6]))
            {
                    currentDirection = CW;
                    if(M_Flags.flag_CW==0)hallCounts++;
                    else hallCounts--;    
            }
            else
            {
                    currentDirection = CCW;
                    if(M_Flags.flag_CW==0)hallCounts--;
                    else hallCounts++;
            }
            calculatePhaseValue(currentSector);
            lastSector = currentSector; /* Update last sector */
	}
}

#ifdef USE_PHASE_INC_AND_CORRECTION
VOID calculatePhaseValue(WORD sectorNo)
{   
   
        phaseOffsetCW = PHASE_OFFSET_CW_750W;
        phaseOffsetCCW = PHASE_OFFSET_CCW_750W;
    #if 1        
    /* Motor commutation is actually based on the required direction, not */
    /* the current dir. This allows driving the motor in four quadrants */    
    if(((controlOutput >= 0) && (requiredDirection == CW)) || ((controlOutput < 0) && (requiredDirection == CCW)))
    {
        tmpQu = __builtin_divmodud((DWORD)sectorNo, (WORD)SECTOR_END, &tmpRe);
        //Use phase offset calculation only when shutter moving up and DC injection not applied
    //    if((requiredDirection == CW) && (!rampStatusFlags.rampDcInjectionOn))
    //        phaseCopy = phase = phaseValues[tmpRe] + phaseOffsetCW;
    //    else
			phaseCopy = phase = phaseValues[tmpRe] + phaseOffsetCW;// + phaseValue;
//        if((requiredDirection == CW) && (!rampStatusFlags.rampDcInjectionOn))
//            phaseCopy = phase = phaseValues[tmpRe] + (phaseOffsetCW + phaseValue);
//        else
//			phaseCopy = phase = phaseValues[tmpRe] + phaseOffsetCW;
        
        phaseIncFlg = PHASE_INCREMENT_FLAG;
    }
    else if(((controlOutput >= 0) && (requiredDirection == CCW)) || ((controlOutput < 0) && (requiredDirection == CW)))
    {
        /* For CCW an offset must be added to compensate difference in */
        /* symmetry of the sine table used for CW and CCW */
        tmpQu = __builtin_divmodud((DWORD)(sectorNo + SECTOR_THREE), (WORD)SECTOR_END, &tmpRe);
        //Use phase offset calculation only when shutter moving up
    //    if((requiredDirection == CW) && (!rampStatusFlags.rampDcInjectionOn))
    //        phaseCopy = phase = phaseValues[tmpRe]+ (phaseOffsetCCW - phaseValue);
    //    else
            phaseCopy = phase = phaseValues[tmpRe] + phaseOffsetCCW;// + phaseValue;
//        if((requiredDirection == CW) && (!rampStatusFlags.rampDcInjectionOn))
//            phaseCopy = phase = phaseValues[tmpRe]+ (phaseOffsetCCW - phaseValue);
//        else
//            phaseCopy = phase = phaseValues[tmpRe]+ phaseOffsetCCW;
        
        
        phaseIncFlg = PHASE_DECREMENT_FLAG;
    }
    #else
    if (requiredDirection == CW)
	{
        phaseCopy = phase = phaseValues[sectorNo] + 2002;
        phaseIncFlg = PHASE_INCREMENT_FLAG;
	}
	else
	{
        // For CCW an offset must be added to compensate difference in 
        // symmetry of the sine table used for CW and CCW
		phaseCopy = phase = phaseValues[(sectorNo + 3) % 6] + 9828; //+ phaseValue;//+ PhaseOffset;
        phaseIncFlg = PHASE_DECREMENT_FLAG;
	}
    #endif
    
}
#else
VOID calculatePhaseValue(WORD sectorNo)
{   
    /* Motor commutation is actually based on the required direction, not */
    /* the current dir. This allows driving the motor in four quadrants */    
    if(((controlOutput >= 0) && (requiredDirection == CW)) || ((controlOutput < 0) && (requiredDirection == CCW)))
    {
        tmpQu = __builtin_divmodud((DWORD)sector, (WORD)SECTOR_END, &tmpRe);
        phase = phaseValues[tmpRe] + phaseOffsetCW;        
    }
    else if(((controlOutput >= 0) && (requiredDirection == CCW)) || ((controlOutput < 0) && (requiredDirection == CW)))
    {
        /* For CCW an offset must be added to compensate difference in */
        /* symmetry of the sine table used for CW and CCW */
        tmpQu = __builtin_divmodud((DWORD)(sector + SECTOR_THREE), (WORD)SECTOR_END, &tmpRe);
        phaseCopy = phase = phaseValues[tmpRe]+ phaseOffsetCCW;
    }
}
#endif
/******************************************************************************
 * measureActualSpeed
 *
 * The measureActualSpeed calculates actual speed of motor from the hall sensor 
 * transistions period. it applies low pass filter to remove noise component.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
VOID measureActualSpeed(VOID)
{
    if(hall2Triggered)
    {
        period = totalTimePeriod;
        totalTimePeriod = 0;
        hall2Triggered = 0;
        cnt_motor_stop = 0;
    }
    
        if (period < MINPERIOD_750W)  
            period = MINPERIOD_750W;
        else if (period > MAXPERIOD_750W)
            period = MAXPERIOD_750W;
    
	periodStateVar+= ((period - periodFilter)*(periodFilterConstant));
	periodFilter = periodStateVar>>15;
        measuredSpeed = __builtin_divud(SPEED_RPM_CALC_750W,periodFilter);
    measuredSpeed_bak=measuredSpeed;
    phaseInc = __builtin_divud(PHASE_INC_CALC,periodFilter);
//    phaseIncPerSec = __builtin_muluu(measuredSpeed,655);
//    phaseInc = __builtin_divud(phaseIncPerSec,1000);
    
#ifdef PHASE_ADVANCE    
    register int a_reg asm("A");
    a_reg = __builtin_mpy(MAX_PH_ADV,SPD_CAL_FOR_PHASEADVANCE , 0,0,0,0,0,0);//SPD_CAL_FOR_PHASEADVANCE
    PhaseAdvance = __builtin_sac(a_reg,0);
    if(requiredDirection == CCW)
        PhaseAdvance = -PhaseAdvance;
#endif        
}

VOID speedControl(VOID) 
{   
    speedPIparms.qInRef = refSpeed;
    speedPIparms.qInMeas = measuredSpeed;

        speedPIparms.qOutMax = 31128;//currentLimitClamp;
        speedPIparms.qOutMin = -(31128);
        CalcPI(&speedPIparms);

        if(flags.speedControl)
            controlOutput = speedPIparms.qOut;  
}



/******************************************************************************
 * initSpeedControllerVariables
 *
 * The initSpeedControllerVariables inititializes all the variables used by 
 * speed control mode.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
VOID intitSpeedController(VOID)
{ 
    
    hall2Triggered = 0;
    totalTimePeriod = 0;
    measuredSpeed = 0; 
    refSpeed = 200;
    
    PhaseAdvance = 0;
    
        controlOutput = 0;
        period = MAXPERIOD_750W;
        periodFilter = MAXPERIOD_750W;
        periodFilterConstant = PERIOD_FILTER_CONST;
        periodStateVar = ((DWORD)MAXPERIOD_750W << 15);
        phaseInc = __builtin_divud(PHASE_INC_CALC, periodFilter);

    tmpQu = 0;
    tmpRe = 0;

    hallValue = HALLA_BIT + (HALLB_BIT << 1) + (HALLC_BIT << 2);    
	sector = sectorTable[hallValue];	//Get sector from table
    lastSector = sector;
    calculatePhaseValue(sector);
    
    //initPiNew(&speedPIparms,P_SPEED_PI_MoteR,I_SPEED_PI_MoteR,C_SPEED_PI,5000,-5000,0);
    speedPIparms.qdSum = 0;
    speedPIparms.qKp = P_SPEED_PI_MoteR;
    speedPIparms.qKi = I_SPEED_PI_MoteR;
    speedPIparms.qKc = C_SPEED_PI;
    speedPIparms.qOutMax = 5000;
    speedPIparms.qOutMin = 5000;
    speedPIparms.qOut = 0;

//        if(requiredDirection == CW)
//        {
////            initPiNew(&speedPIparms,P_SPEED_PI_CW_750W,I_SPEED_PI_CW_750W,C_SPEED_PI,currentLimitClamp,-(currentLimitClamp),0);
//            initPiNew(&speedPIparms,P_SPEED_PI_CW_750W,I_SPEED_PI_CW_750W,C_SPEED_PI,5000,-5000,0);
//        }
//        else
//        {
//            initPiNew(&speedPIparms,P_SPEED_PI_CCW_750W,I_SPEED_PI_CCW_750W,C_SPEED_PI,5000,-5000,0);
//        }

}


