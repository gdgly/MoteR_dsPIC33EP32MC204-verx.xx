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

/*  define MCU type   */ 
#define MCU_dsPIC33EPxxxGM3xxx
//#define MCU_dsPIC33EPxxxMC2xxx

/*  define IGBT type   */ 
//#define IGBT_LowActive
#define IGBT_HighActive

#undef PHASE_ADVANCE	// for extended speed ranges this should be defined

#define USE_PHASE_INC_AND_CORRECTION

#define ENABLE      1
#define DISABLE     0

#define CW	0		/* Counter Clock Wise direction */
#define CCW	1		/* Clock Wise direction */


#define NO_POLEPAIRS_MoteR  4

#define MAX_RPM         8000
#define MIN_RPM         50//100

#define FCY  70000000UL	 // xtal = 8Mhz; with PLL -> 70 MIPS
#define FPWM 16000        //20000		 // 20 kHz, so that no audible noise is present.

#define PDIV_256        256  //Timer3 configuration for using prescalar 256 (1ms) Tcy/256
#define PDIV_64         64   //Timer3 configuration for using prescalar 64 (1us)  Tcy/64 
 
#define PDIV    PDIV_256 //Current timer3 configuration

/* for speed rpm calculation */
#define SPEED_RPM_CALC_MoteR      ((((DWORD)FCY*60)/(PDIV*2*4)))   

// Period Calculation
// Period = (FCY / PDIV * 60) / (RPM * NO_POLEPAIRS )
#define MINPERIOD_MoteR	((DWORD)((FCY/PDIV)*60)/((DWORD)MAX_RPM*2*4))  
#define MAXPERIOD_MoteR	((DWORD)((FCY/PDIV)*60)/((DWORD)MIN_RPM*2*4))  

#define PHASE_INC_CALC 1792000UL //((DWORD)(FCY/((DWORD)PDIV*2*FPWM))*65536) //For 5Khz

/* Half of the PWM Deadtime; The effective deadtime written in ALTDTR registers is 2*ALTDTR_DIV2 */
#define	ALTDTR_DIV2_MoteR	140//120
//#define	ALTDTR_DIV2	147//(70*2.1)      //2.1us

/* Use this MACRO when using floats to initialize signed 16-bit fractional variables */
#define Q15(Float_Value)	\
        ((Float_Value < 0.0) ? (SHORT)(32768 * (Float_Value) - 0.5) \
        : (SHORT)(32767 * (Float_Value) + 0.5))


#endif
