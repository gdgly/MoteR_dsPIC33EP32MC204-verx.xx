/******************************************************************************************/
/*  FILE        :main_SensoredBLDC.c                                                      */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

//#include <p33Fxxxx.h>
#include <p33Exxxx.h>
#include "defs_ram.h"
#include "Init.h"
#include "SensoredBLDC.h"
/******************************************************************************/
/* Configuration bits                                                         */
/******************************************************************************/

_FOSCSEL(FNOSC_FRC & IESO_OFF & PWMLOCK_OFF);	 // Start with FRC will switch to Primary (XT, HS, EC) Oscillator with PLL
_FOSC(FCKSM_CSECMD & POSCMD_XT);	// Clock Switching Enabled and Fail Safe Clock Monitor is disable
    				        // Primary Oscillator Mode: XT Crystal
_FPOR(ALTI2C1_OFF & ALTI2C2_OFF & WDTWIN_WIN50);

_FWDT(PLLKEN_ON & FWDTEN_OFF);
/* Turn off Watchdog Timer */

_FGS(GWRP_OFF & GCP_OFF);
/* Set Code Protection Off for the General Segment */

_FICD (ICS_PGD2 & JTAGEN_OFF);
/* Use PGC2/PGD2 for programming and debugging */





int main(void)
{
        System_Clock_Init();

        GPIO_Init();
	InitADC10();
	InitTMR1();
	InitTMR3();
	timer3avg = 0;
	InitMCPWM();
	InitIC();
                              /*15k---->950
                                20k---->cw  Direction=0  650  3000rpm
                                      ccw  Direction=1  750  3000rpm*/
        AD_SET_SPEED=650;//950;
	while(1)
	{
#ifndef CLOSEDLOOP
           runTestCode();  /* Run test code */
           adc_IBUS();
#endif
	}
}

