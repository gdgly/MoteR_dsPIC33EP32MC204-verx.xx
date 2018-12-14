/*********************************************************************************
* FileName: GPIO.c
* Description:
* This source file contains the definition of all the functions for GPIO.
* It initializes all the port pins required in application.
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
*****************************************************************************/
#include <p33Exxxx.h>
#include "GPIO.h"
#include "./Common/UserDefinition/Userdef.h"

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

/*
Added for motor control board hardware version 2 on 30 Dec 2014
*/
	/*	Unlock the Reprogrammable Pin Mechanism */
	__builtin_write_OSCCONL(OSCCON & (~(1<<6))); /* clear bit 6  */
    
    /*PWM Initialisation starts*/
	TRISBbits.TRISB10= 0;//PWM3H
	TRISBbits.TRISB11= 0;//PWM3L
	TRISBbits.TRISB12= 0;//PWM2H
	TRISBbits.TRISB13= 0;//PWM2L
	TRISBbits.TRISB14= 0;//PWM1H
	TRISBbits.TRISB15= 0;//PWM1L
    
	TRISEbits.TRISE13 = 1;/*FLT6*/
	ANSELEbits.ANSE13 = 0;
    CNPUEbits.CNPUE13 = 1; 
    
    PORTGbits.RG9 = 1;//disable PWM buffer
	TRISGbits.TRISG9= 0;//PWM enable
	ANSELGbits.ANSG9 = 0;//Disable Analog input on PWM enable pin
    
	ANSELB = 0x0000;// Disable Analog input on PWM channels
	/*PWM initialisation ends*/
	
	/*	Sensor interface to motor drive board start*/
	TRISAbits.TRISA11 = 1;	/*Air Switch,Wrap around sensor,Micro Switch sensor*/
    CNPUAbits.CNPUA11 = 1; 
	TRISDbits.TRISD8 = 1;	/*origin sensor*/
    CNPUDbits.CNPUD8 = 1; 
	
	//	Added on 20Feb2015 for IGBT over temperature fault
	TRISBbits.TRISB4 = 1;	/*IGBT over temperature fault*/
    CNPUBbits.CNPUB4 = 1;
	
	// For testing only (Added on 27 Jan 2015 to enable MCU_LATCH_CTRL output)
	/**************************************************/
	ANSELGbits.ANSG8 = 0;	//Disable Analog input on MCU_LATCH_CTRL fault latch clear bit
	TRISGbits.TRISG8 = 0;	/*MCU_LATCH_CTRL fault latch clear bit*/
    CNPUGbits.CNPUG8 = 1;
	PORTGbits.RG8 = 1;		//	Set HIGH as default value for MCU_LATCH_CTRL
	/**************************************************/
	
	TRISCbits.TRISC8 = 0;//enable brake
	TRISCbits.TRISC13 = 0;//enable fan
    PORTCbits.RC13 = 0;
	
	ANSELAbits.ANSA12 = 0;/*Photoelec Obs sensor */
	ANSELAbits.ANSA11 = 0;/*Air Switch,Wrap around sensor,Micro Switch sensor*/
	ANSELBbits.ANSB7 = 0;/*Power fail Occured*/
	ANSELEbits.ANSE15 = 0;/*Temperature sensor*/
	
	/*HAll sensor configuration starts*/
	TRISAbits.TRISA8 = 1;
    CNPUAbits.CNPUA8 = 1; 
	TRISCbits.TRISC6 = 1;
    CNPUCbits.CNPUC6 = 1; 
	RPINR7 = 0;
	RPINR7bits.IC1R = 0x18;/*  Assign IC1(HALLA) to RPI24 */
	RPINR7bits.IC2R = 0x36;/*  Assign IC2(HALLB) to RP54 */
    TRISFbits.TRISF0 = 1;
    CNPUFbits.CNPUF0 = 1; 
	RPINR8 = 0;
	RPINR8bits.IC3R = 0x60;/*  Assign IC3(HALLC) to RPI96 */
	
	//	Added on 20Feb2015 for IGBT over temperature fault
	/* Configure interrupt 1 for over temperature*/
	INTCON2bits.INT1EP = 0;		//	Interrupt on positive edge as default value of this line is low in HW ver 2.
	IEC1bits.INT1IE = 1;		//	Enable INT 1 interrupt.
    IPC5bits.INT1IP = 7;		//	Interrupt priority (highest)
	IFS1bits.INT1IF = 0;		//	Clear interrupt flag.
	RPINR0 = 0;					//	Clear PERIPHERAL PIN SELECT INPUT REGISTER 0 before selecting INT1 functionality on RP36
	RPINR0bits.INT1R = 0x24;	//  Assign INT1 to RPI36
	/*	Sensor interface to motor drive board ends*/
		
	/* Analog inputs configuration starts*/
	TRISEbits.TRISE14 = 1; /*BLDC Curent Monitoring, common current in all legs*/
	ANSELEbits.ANSE14 = 1; /*Itotal current*/	
	TRISBbits.TRISB0 = 1; /*Boost Converter O/P Voltage monitor*/
	ANSELBbits.ANSB0 = 1; /*VBUS/DC_BUS*/
	    
	__builtin_write_OSCCONL(OSCCON | (1<<6)); 	 /* Set bit 6 */

}
