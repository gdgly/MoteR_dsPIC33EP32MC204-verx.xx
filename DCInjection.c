/******************************************************************************************/
/*  FILE        :uart.c                                                                   */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

//#include <p33Fxxxx.h>
#include <p33Exxxx.h>
#include "defs_ram.h"
#include "Init.h"

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
void DCInjectionON(void)
{
    IOCON1 = 0XC200; 
    IOCON2 = 0XC200; 
    IOCON3 = 0XC200;    
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
void DCInjectionOFF(void)
{ 
    
}