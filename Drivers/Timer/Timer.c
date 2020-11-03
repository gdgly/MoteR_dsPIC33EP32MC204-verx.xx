/*********************************************************************************
* FileName: Timer.c
* Description:
* This source file contains the definition of all the functions for Timer.
* It initializes all the timers required in application
**********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#include <p33Exxxx.h>
#include <string.h>
#include "Timer.h"
#include "./Common/UserDefinition/Userdef.h"

/******************************************************************************
 * initTMR1
 *
 * This function initializes TIMER1 for speed control and motor stalled protection.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
VOID initTMR1(VOID)
{
	T1CON = 0x0030;			/* internal Tcy/256 clock */
	TMR1 = 0;
	PR1 = 273;				/* 1 ms timer */
    IPC0bits.T1IP = 5;
}
/******************************************************************************
 * initTMR2
 *
 * This function initializes TIMER2 for current control of motor.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
VOID initTMR2(VOID)
{
	T2CON = 0x0030;			/* internal Tcy/256 clock */
	TMR2 = 0;
	PR2 = 273;				/* 1 ms timer */
    IPC1bits.T2IP = 3;
}
/******************************************************************************
 * initTMR3
 *
 * This function initializes TIMER3 used as a timebase for the two input capture channels.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
VOID initTMR3(VOID)
{
#if (PDIV == PDIV_256)
    T3CON = 0x0030; /* internal Tcy/256 clock */
#elif (PDIV == PDIV_64)
    T3CON = 0x0020; /* internal Tcy/64 clock */
#else
    #error Timer3 not configured
#endif    
	TMR3 = 0;
	PR3 = 0xFFFF;
    
    IFS0bits.T3IF = 0;
    IPC2bits.T3IP = 1;
}

/******************************************************************************
 * initTMR5
 *
 * This function initializes Timer5 used to app
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
VOID initTMR5(VOID)
{
	T5CON = 0x0030;	 /* internal Tcy/256 clock */
	TMR5 = 0;
	PR5 = 2734;				/* 10 ms timer */
	IFS1bits.T5IF = 0;  /* Clear timer 8 flag */
	IEC1bits.T5IE = 1;	/* Enable interrupts for Timer 8 */
    T5CONbits.TON = 1; //stop timer
    IPC7bits.T5IP = 1;
}



