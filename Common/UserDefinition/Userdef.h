/********************************************************************************
* FileName: Userdef.h
* Description:  
* This header file contains the common macros used by different modules.
*********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#ifndef USERDEF_H
#define USERDEF_H
#include "./Common/Typedefs/Typedefs.h"

#undef PHASE_ADVANCE	// for extended speed ranges this should be defined

#define USE_PHASE_INC_AND_CORRECTION
#define PICOMO_MOTOR        1//not use
#define NEW_MOTOR_1         2//not use
#define NEW_MOTOR_2         3//750W motor
#define NEW_MOTOR_3         4
#define MOTOR_TYPE      NEW_MOTOR_2//PICOMO_MOTOR
/*
	Added for motor control board hardware version 2 on 17 Dec 2014
*/
/*********************************************************************
					Macros for motror control board
					hardware selection begin here
*********************************************************************/
#define		MCHV2BOARD				1		//	Microchip kit
#define		MTR_CTRL_HW_VER1		2		//	Motor control board version 1
#define		MTR_CTRL_HW_VER2		3		//	Motor control board version 2

#define		MTR_CTRL_HW_TYPE		MTR_CTRL_HW_VER2	//	Motor control board version 1

#define     SVM_INTERRUPT_USE_PWM

/*********************************************************************
					Macros for motror control board
					hardware selection end here
*********************************************************************/

#define ENABLE      1
#define DISABLE     0

//	Added on 3 Feb 2015 to implement user control on power on calibration
#define SERVICED	0
#define	INITIATED	1
#define	TERMINATED	2

#define CW	0		/* Counter Clock Wise direction */
#define CCW	1		/* Clock Wise direction */


#define NO_POLEPAIRS_750W  4

#define MAX_RPM         8000//4800
#define MIN_RPM         50//10//100

#define FCY  70000000UL	 // xtal = 8Mhz; with PLL -> 70 MIPS
#define FPWM 16000        //20000		 // 20 kHz, so that no audible noise is present.

#define PDIV_256        256  //Timer3 configuration for using prescalar 256 (1ms) Tcy/256
#define PDIV_64         64   //Timer3 configuration for using prescalar 64 (1us)  Tcy/64 
 
#if (MOTOR_TYPE == PICOMO_MOTOR)
    #define PDIV    PDIV_64 //Current timer3 configuration
#elif (MOTOR_TYPE == NEW_MOTOR_1)
    #define PDIV    PDIV_256 //Current timer3 configuration
#elif (MOTOR_TYPE == NEW_MOTOR_2)
    #define PDIV    PDIV_256 //Current timer3 configuration
#else
    #error Timer3 prescalare not defined
#endif

/* for speed rpm calculation */
#define SPEED_RPM_CALC_750W      ((((DWORD)FCY*60)/(PDIV*2*4)))   

// Period Calculation
// Period = (FCY / PDIV * 60) / (RPM * NO_POLEPAIRS )
#define MINPERIOD_750W	((DWORD)((FCY/PDIV)*60)/((DWORD)MAX_RPM*2*4))  
#define MAXPERIOD_750W	((DWORD)((FCY/PDIV)*60)/((DWORD)MIN_RPM*2*4))  

#define PHASE_INC_CALC 1792000UL //((DWORD)(FCY/((DWORD)PDIV*2*FPWM))*65536) //For 5Khz

/* Half of the PWM Deadtime; The effective deadtime written in ALTDTR registers is 2*ALTDTR_DIV2 */
#define	ALTDTR_DIV2_750W	140//120
//#define	ALTDTR_DIV2	147//(70*2.1)      //2.1us

/* Use this MACRO when using floats to initialize signed 16-bit fractional variables */
#define Q15(Float_Value)	\
        ((Float_Value < 0.0) ? (SHORT)(32768 * (Float_Value) - 0.5) \
        : (SHORT)(32767 * (Float_Value) + 0.5))


#endif
