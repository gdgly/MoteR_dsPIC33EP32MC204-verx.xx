/********************************************************************************
* FileName: GPIO.h
* Description:  
* This header file contains the decleration of all the attributes and 
* services for GPIO.c file. It initializes all the required port pins.
*********************************************************************************/

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
#ifndef GPIO_H
#define GPIO_H
#include "./Common/Typedefs/Typedefs.h"

/* This function initialize all I/O's */
VOID initGPIO(VOID);




    #define Out_PWM3H_dir                 TRISBbits.TRISB10
    #define Out_PWM3L_dir                 TRISBbits.TRISB11
    #define Out_PWM2H_dir                 TRISBbits.TRISB12
    #define Out_PWM2L_dir                 TRISBbits.TRISB13
    #define Out_PWM1H_dir                 TRISBbits.TRISB14
    #define Out_PWM1L_dir                 TRISBbits.TRISB15
    #define In_FLT1_dir                   TRISBbits.TRISB8
    #define HALLA_dir                     TRISBbits.TRISB1
    #define HALLB_dir                     TRISBbits.TRISB2
    #define HALLC_dir                     TRISBbits.TRISB3

    #define In_STOP_dir                   TRISAbits.TRISA8
    #define In_OPEN_dir                   TRISCbits.TRISC2
    #define In_CLOSE_dir                  TRISCbits.TRISC1 

    #define Out_BRAKE_dir                 TRISCbits.TRISC9

    #define In_ADC_TMEP_dir               TRISAbits.TRISA0     
    #define In_ADC_IBUS_dir               TRISAbits.TRISA1
    #define In_ADC_VBUS_dir               TRISCbits.TRISC0
    #define Out_LED_PGD_dir               TRISBbits.TRISB5
                    //=======================================
    #define In_STOP                       PORTAbits.RA8
    #define In_OPEN                       PORTCbits.RC2
    #define In_CLOSE                      PORTCbits.RC1
    #define Out_LED_PGD                   LATBbits.LATB5

#endif /* GPIO_H */
