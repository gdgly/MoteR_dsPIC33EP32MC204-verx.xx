/*********************************************************************************
* FileName: GPIO.c
* Description:
* This source file contains the definition of all the functions for GPIO.
* It initializes all the port pins required in application.
**********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#include <p33Exxxx.h>
#include "GPIO.h"
#include "./Common/UserDefinition/Userdef.h"
#include "./Application/RampGenerator/RampGenerator.h"

/******************************************************************************
 * initGPIO
 *
 * This function initializes all the I/O's required by application.
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
VOID initGPIO(VOID)
{

    ANSELA = 0x0000; // Turn off all ADC analog intput pins
    ANSELB = 0x0000;
    ANSELC = 0x0000;
	TRISA  = 0X00;	//all bits is 0
	TRISB  = 0X00;	
	TRISC  = 0X00;	
	PORTA  = 0X00;	
	PORTB  = 0X00;	
	PORTC  = 0X00;	
	LATA   = 0X00;	
	LATB   = 0X00;	    
	LATC   = 0X00;		
	ODCA   = 0X00;	
	ODCB   = 0X00;	
	ODCC   = 0X00;	
	CNENA  = 0X00;	
	CNENB  = 0X00;	
	CNENC  = 0X00;	
    CNPUA=0xFFFF;
    CNPUB=0xFFFF;
    CNPUC=0xFFFF;
    CNPUBbits.CNPUB10=0;
    CNPUBbits.CNPUB11=0;
    CNPUBbits.CNPUB12=0;
    CNPUBbits.CNPUB13=0;
    CNPUBbits.CNPUB14=0;
    CNPUBbits.CNPUB15=0;
        
/*
Added for motor control board hardware version 2 on 30 Dec 2014
*/
	/*	Unlock the Reprogrammable Pin Mechanism */
	__builtin_write_OSCCONL(OSCCON & (~(1<<6))); /* clear bit 6  */
    
    /*PWM Initialisation starts*/
        Out_PWM1H_dir=0;
        Out_PWM1L_dir=0;
        Out_PWM2H_dir=0;
        Out_PWM2L_dir=0;
        Out_PWM3H_dir=0;
        Out_PWM3L_dir=0;     
        In_FLT1_dir=1;
        RPINR12bits.FLT1R = 40;      //flt1 to pin rp40   
	/*HAll sensor configuration starts*/
        HALLA_dir = 1;
        HALLB_dir = 1;
        RPINR7 = 0;
        RPINR7bits.IC1R = 35;//33;/*  Assign IC1(HALLA) to RPI33 */
        RPINR7bits.IC2R = 34;//34;/*  Assign IC2(HALLB) to RPI34 */
        HALLC_dir = 1;
        RPINR8 = 0;
        RPINR8bits.IC3R = 33;//35;/*  Assign IC3(HALLC) to RP35 */
    /*UART configuration*/
        RPOR6bits.RP54R=1;         //Set RP54 RC6-->U1TX
        RPINR18bits.U1RXR=32;         //Set U1RX-->RPI32 RB0    
        
        In_STOP_dir=1;
        In_OPEN_dir=1;
        In_CLOSE_dir=1;
	
	    Out_BRAKE_dir = 0;//enable brake
        lockRelease_OUT=0;
		
	/* Analog inputs configuration starts*/
	In_ADC_IBUS_dir = 1; /*BLDC Curent Monitoring, common current in all legs*/
	ANSELAbits.ANSA1 = 1; /*Itotal current AN1*/	 
    CNPUAbits.CNPUA1=0;
	In_ADC_VBUS_dir = 1; /*Boost Converter O/P Voltage monitor*/
	ANSELCbits.ANSC0 = 1; /*VBUS/DC_BUS AN6*/
    CNPUCbits.CNPUC0=0;
	    
	__builtin_write_OSCCONL(OSCCON | (1<<6)); 	 /* Set bit 6 */

        Out_DBR_CTRL_dir=0;
        Out_DBR_CTRL=0;    
        Out_LED_PGD_dir=0;
        Out_LED_PGD=0;    
        
}
