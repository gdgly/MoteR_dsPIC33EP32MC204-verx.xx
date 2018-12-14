/*********************************************************************************
* FileName: Main.c
* Description:
* This source file contains the definition of all the functions for Main.
* It configures the CPU and initializes all the peripherals required in
* application.
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
 *  22/04/2014			  iGate 		 Updated - Added Logic Solver
*****************************************************************************/
#include <p33Exxxx.h>
#include "math.h"
#include "./Common/UserDefinition/Userdef.h"
#include "./Drivers/ADC/ADC.h"
#include "./Drivers/InputCapture/InputCapture.h"
#include "./Drivers/Timer/Timer.h"
#include "./Drivers/PWM/MCPWM.h"
#include "./Drivers/GPIO/GPIO.h"
#include "./Application/RampGenerator/RampGenerator.h"
#include "./Application/Application.h"
#include "./Common/Typedefs/Typedefs.h"
#include "./MotorControl/CurrentController/CurrentController.h"
#include "./Common/Delay/Delay.h"
#include "./Common/Extern/Extern.h"

/******************************************************************************/
/* Configuration bits                                                         */
/******************************************************************************/
_FPOR(ALTI2C1_OFF & ALTI2C2_OFF & BOREN_OFF);
_FWDT(WDTPOST_PS1024 & WDTPRE_PR32 & PLLKEN_ON & WINDIS_OFF & FWDTEN_ON);
_FOSCSEL(FNOSC_FRC & IESO_OFF & PWMLOCK_OFF);
_FGS(GWRP_OFF & GCP_OFF);
_FICD(ICS_PGD2 & JTAGEN_OFF);	/* PGD3 for 28pin 	PGD2 for 44pin */
_FOSC(FCKSM_CSECMD & POSCMD_XT);		/* XT W/PLL */


#define     Reset()     {__asm__   volatile ("reset");}
/******************************************************************************
 * main
 *
 * The main function gives starting point for application execution. It configures
 * the CPU and initializes all the peripherals required in application.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
INT main(VOID)
{
    /*********************************************************************
    * The settings below set up the oscillator and PLL for 70 MIPS as
    * follows:
    *             Crystal Frequency  * (DIVISOR+2)
    *  Fcy =     ---------------------------------
    *               PLLPOST * (PRESCLR+2) * 4
	*  Crystal  = 16 MHz
	*  Fosc		= 140 MHz
	*  Fcy		= 70 MIPS
    *********************************************************************/
	PLLFBD = 68;		        /* M=70 */
	CLKDIVbits.PLLPOST = 0;		/* N1=2 */
	CLKDIVbits.PLLPRE = 2;		/* N2=4 */
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(0x01);
	while(OSCCONbits.COSC != 0b011);
	while(OSCCONbits.LOCK != 1);/* Wait for PLL to lock */

	CORCONbits.SATA = 0;
    CORCONbits.IF = 0;
    //Add powerup delay
    delayMs(250);
	ClrWdt();   // clear the WDT to inhibit the device reset
    
    initGPIO(); /* Initialize all the I/O's required in application */
    
	initADC();		/* Initialize ADC to be signed fractional */
	//Added for ADC2- RN- NOV2015
    initADC2();		/* Initialize ADC to be signed fractional */
	initInputCapture();		/* Initialize Hall sensor inputs ISRs	 */
	initTMR1();		/* Initialize TMR1 1 ms periodic ISR for speed controller */
    initTMR2();		/* Initialize TMR2 1 ms periodic ISR for current controller */
    initTMR3();		/* Initialize TMR3 for timebase of capture */
    initTMR5();
	ClrWdt();   // clear the WDT to inhibit the device reset
	initMCPWM();
    initTMR9();
    
    measureADCOffset();
	ClrWdt();   // clear the WDT to inhibit the device reset

	// For testing only (Added on 27 Jan 2015 to enable fault input)
	// These lines were disabled in initMCPWM() function called above
	// in MCPWM.c file.
	/**************************************************/
	PWMCON1bits.FLTIEN = 1; /* Enable fault interrupt */
	FCLCON1 = 0x0004;
	FCLCON2 = 0x0004;
	FCLCON3 = 0x0004;

	
	AD2CON1bits.ADON = 1;
    T5CONbits.TON = 1;
	for(;;)
	{
        ClrWdt();   // clear the WDT to inhibit the device reset
        application();		
    }
    return 0; /* Return without error */
}
