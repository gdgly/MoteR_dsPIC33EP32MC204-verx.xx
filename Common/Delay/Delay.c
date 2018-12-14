/*********************************************************************************
* FileName: Delay.c
* Description:
* This source file contains the definition of all the functions for delay routines.
**********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/

#define FCY 70000000UL
#include <libpic30.h>
#include "Delay.h"

/******************************************************************************
 * delayUs
 *
 * The delayUs function inserts 1us delay.
 *
 * PARAMETER REQ: number of microseconds to delay
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
VOID delayUs(WORD delayCount)
{
    __delay_us(delayCount);
}

/******************************************************************************
 * delayMs
 *
 * The delayMs function inserts 1ms delay.
 *
 * PARAMETER REQ: number of milliseconds to delay
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/ 
VOID delayMs(WORD delayCount)
{
    __delay_ms(delayCount);
}
