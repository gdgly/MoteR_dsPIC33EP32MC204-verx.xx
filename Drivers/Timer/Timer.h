/********************************************************************************
* FileName: Timer.h
* Description:  
* This header file contains the decleration of all the attributes and 
* services for Timer.c file. It initializes all the timers required in application
*********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#ifndef TIMER_H
#define TIMER_H
#include "./Common/Typedefs/Typedefs.h"

/* TIMER1 used for speed control and motor stalled protection */
VOID initTMR1(VOID);

/* TIMER2 used for current control of motor */
VOID initTMR2(VOID);

/* TIMER3 used as a timebase for the two input capture channels */
VOID initTMR3(VOID);
/*This function initializes Timer5 used to app*/
VOID initTMR5(VOID);

#endif /* TIMER_H */
