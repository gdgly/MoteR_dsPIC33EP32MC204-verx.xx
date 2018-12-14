/******************************************************************************************/
/*  FILE        :Init.c                                                                   */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/
//#include <p33Fxxxx.h>
#include <p33Exxxx.h>
#include "defs_ram.h"

void lockIO(void);
void unlockIO(void);


/*********************************************************************
  Function:        void System_Clock_Init(void)

  Overview:        intializes the System Clock

  Note:            None.
********************************************************************/
void System_Clock_Init(void)
{
	// Configure Oscillator to operate the device at **Mhz
	// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2

#if defined(__Fin16MHz_Fosc140MHz__)
        /****************************************************/
        PLLFBD =  68;    // M=70    Fosc= 16*70/(2*4)= 140Mhz for 16M input clock
	CLKDIVbits.PLLPOST = 0;		// N2=2
	CLKDIVbits.PLLPRE = 2;		// N1=4
         /***************************************************/

	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(0x01);

	while(OSCCONbits.COSC != 0b011);
	// Wait for PLL to lock
	while(OSCCONbits.LOCK != 1);
#endif

#if defined(__Fin8MHz_Fosc20MHz__)
        /****************************************************/
         PLLFBD =  8;    // M=10   Fosc= 8*10/(2*2)= 20Mhz for 8M input clock
         CLKDIVbits.PLLPOST = 0;	// N2=2
         CLKDIVbits.PLLPRE = 0;		// N1=2
        /***************************************************/

	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(0x01);

	while(OSCCONbits.COSC != 0b011);
	// Wait for PLL to lock
	while(OSCCONbits.LOCK != 1);
#endif
}


/*********************************************************************
  Function:        void GPIO_Init(void)

  Overview:        intializes the GPIO

  Note:            None.
********************************************************************/
void GPIO_Init(void)
{
        ANSELA = 0x0000; // Turn off all ADC analog intput pins
        ANSELB = 0x0000;
        ANSELC = 0x0000;

	TRISA  = 0X00;	//???????????
	TRISB  = 0X00;	//???????????
	TRISC  = 0X00;	//???????????

	PORTA  = 0X00;	//???????0
	PORTB  = 0X00;	//???????0
	PORTC  = 0X00;	//???????0

	LATA   = 0X00;	//???????0
	LATB   = 0X00;	//???????0    
	LATC   = 0X00;	//???????0
	
	ODCA   = 0X00;	//?????????0
	ODCB   = 0X00;	//?????????0
	ODCC   = 0X00;	//?????????0

	CNENA  = 0X00;	//?????????????
	CNENB  = 0X00;	//?????????????
	CNENC  = 0X00;	//?????????????


//        CNPUA=0x0000;
//        CNPUB=0x0000;
//        CNPUC=0x0000;
//        CNPUBbits.CNPUB1=1;
//        CNPUBbits.CNPUB2=1;
//        CNPUBbits.CNPUB3=1;
//        CNPUBbits.CNPUB4=1;
//        CNPUCbits.CNPUC1=1;
//        CNPUCbits.CNPUC2=1;
//        CNPUAbits.CNPUA8=1;
//        CNPUAbits.CNPUA4=1;
//        CNPUAbits.CNPUA9=1;


        CNPUA=0xFFFF;
        CNPUB=0xFFFF;
        CNPUC=0xFFFF;
        CNPUBbits.CNPUB10=0;
        CNPUBbits.CNPUB11=0;
        CNPUBbits.CNPUB12=0;
        CNPUBbits.CNPUB13=0;
        CNPUBbits.CNPUB14=0;
        CNPUBbits.CNPUB15=0;

	unlockIO();
        RPINR12bits.FLT1R = 40;         //flt1 to pin rp40
	RPINR7bits.IC1R = 33;		// IC1 on RP1/RB1
	RPINR7bits.IC2R = 34;		// IC2 on RP2/RB2
	RPINR8bits.IC3R = 35;	        // IC3 on RP3/RB3
#if defined(__RPI32_UART_SELCET__)
        RPINR18bits.U1RXR=32;
#endif
	lockIO();
        In_IBUS_dir=1;
        In_HALL_U_dir=1;
        In_HALL_V_dir=1;
        In_HALL_W_dir=1;
        In_FLT1_dir=1;
        Out_PWM1H_dir=0;
        Out_PWM1L_dir=0;
        Out_PWM2H_dir=0;
        Out_PWM2L_dir=0;
        Out_PWM3H_dir=0;
        Out_PWM3L_dir=0;          

        In_LOW_LIM_dir=1;
        In_STOP_dir=1;
        In_OPEN_dir=1;
        In_CLOSE_dir=1;
        In_PHOTO_LED2_dir=1;
        In_PHOTO_LED1_dir=1;
        In_SENSOR_dir=1;
        In_MSS1_dir=1;
        In_MSS2_dir=1;
        In_COLUMN_dir=1;
        In_SCREEN_dir=1;
        In_TEMP_PROTECT_dir=1;

        Out_ENABLE_BRAKE_dir=0;
        Out_ENABLE_BRAKE=0;
        Out_RELAY_DOWN_dir=0;
        Out_RELAY_DOWN=0;
        Out_RELAY_UP_dir=0;
        Out_RELAY_UP=0;
        Out_RELAY_DOWN_LIM_dir=0;
        Out_RELAY_DOWN_LIM=0;
        Out_RELAY_UP_LIM_dir=0;
        Out_RELAY_UP_LIM=0;

        Out_LED_PGD_dir=0;
        Out_LED_PGD=1;
}
/*******************************************************************
		Below is the code required to setup the ADC registers for :
		1. 1 channel conversion (in this case AN8)
		2. PWM trigger starts conversion
		3. Pot is connected to CH0 and AN8
		4. Manual Stop Sampling and start converting
		5. Manual check of Conversion complete 
																
*********************************************************************/
void InitADC10(void)
{
        ANSELAbits.ANSA0 = 1;		// AN0 for ibus
        ANSELBbits.ANSB0 = 1;		// AN2 For ZB_AD
        CNPUAbits.CNPUA0=0;

 	/* set channel scanning here, auto sampling and convert, 
 	   with default read-format mode */
	AD1CON1 = 0x006C;
	/* select 10-bit, 1 channel ADC operation */
        AD1CON1bits.SSRC = 0;//3;
	AD1CON1bits.SSRCG = 1;
	AD1CON1bits.AD12B = 0;
        AD1CON1bits.FORM = 0;
        AD1CON1bits.SIMSAM = 1 ;       //Samples CH0, CH1 simultaneously

        AD1CON4 = 0x0000;	//no dma usage
	/* No channel scan for CH0+, Use MUX A,  
	   SMPI = 1 per interrupt, Vref = AVdd/AVss */
	AD1CON2 = 0x0000;
	
	/* Set Samples and bit conversion time */
	//AD1CON3 = 0x032F;
        AD1CON3 = 0x0005;

	AD1CSSL = 0x0000;
	
	/* channel select AN2 */
	AD1CHS0 = 0x0002;     //AN2-->ch0

        AD1CON2bits.CHPS=1;   //convert ch0,ch1
        AD1CHS123bits.CH123SA= 0;   //AN0-->ch1

        AD1CON1bits.DONE = 0;	//Making sure that there is not any conversion in progress
	IPC3bits.AD1IP = 5;		//Assigning ADC ISR priority
	IFS0bits.AD1IF = 0;           
	IEC0bits.AD1IE = 1;       
	AD1CON1bits.ADON = 1;      
}

/********************************************************************
InitMCPWM, intializes the PWM as follows:
1. FPWM = 39000 hz
2. Independant PWMs
3. Control outputs using OVDCON
4. Set Duty Cycle with the ADC value read from pot
5. Set ADC to be triggered by PWM special trigger
*********************************************************************/

void InitMCPWM(void)
{
// ============= Motor PWM ======================

//互补模式，独立占空比和相位，固定主周期，边沿对齐
	PTCON  = 0X0000;   	//DIS PWM MODULE
	PTPER  = FCY/FPWM-1;//1250;590;    	//SET PWM PERIOD ONPRIMARY TIME BASE

	PHASE1 = 0;       	//SET PHASE SHIFT
	PHASE2 = 0;
	PHASE3 = 0;

	PDC1   = 0;		//SET DUTY CYCLES
	PDC2   = 0;
	PDC3   = 0;

	DTR1   =  DTR2  =  DTR3 = 210;//34;			////SET DEAD TIME VALUES     15--1.7US
	ALTDTR1 = ALTDTR2 = ALTDTR3  = 210;//34;

	IOCON1 = IOCON2 = IOCON3 = 0xc301;//0Xc000;		//SET PWM MODE TO COMPLEMENTRY

	PWMCON1 = PWMCON2 = PWMCON3 = 0X0000; 	//SET PRIMARY TIME BASE ,EDGE-ALIGNED MODE AND INDEPENDENT DUTY CYCLES
	FCLCON1 = FCLCON2 = FCLCON3 = 0X0000; 	//CONFIGURE FAULTS

	FCLCON1bits.FLTSRC = FCLCON2bits.FLTSRC = FCLCON3bits.FLTSRC = 0;//故障源1
	FCLCON1bits.CLSRC = FCLCON2bits.CLSRC = FCLCON3bits.CLSRC = 0;	 //限流源1

	FCLCON1bits.FLTPOL = FCLCON2bits.FLTPOL = FCLCON3bits.FLTPOL = 1; //故障源为低电平有效
	FCLCON1bits.CLPOL = FCLCON2bits.CLPOL = FCLCON3bits.CLPOL = 1;    //限流源为低电平有效

	FCLCON1bits.FLTMOD = FCLCON2bits.FLTMOD = FCLCON3bits.FLTMOD = 1;//
	FCLCON1bits.CLMOD = FCLCON2bits.CLMOD = FCLCON3bits.CLMOD = 1;//

	LEBCON1bits.PHR = LEBCON2bits.PHR=LEBCON3bits.PHR=1;
	LEBCON1bits.PHF = LEBCON2bits.PHF=LEBCON3bits.PHF=0;
	LEBCON1bits.PLR = LEBCON2bits.PLR=LEBCON3bits.PLR=1;
	LEBCON1bits.PLF = LEBCON2bits.PLF=LEBCON3bits.PLF=0;
	LEBCON1bits.FLTLEBEN = LEBCON2bits.FLTLEBEN=LEBCON3bits.FLTLEBEN=1;
	LEBCON1bits.CLLEBEN = LEBCON2bits.CLLEBEN=LEBCON3bits.CLLEBEN=1;

	PWMCON1bits.FLTIEN = PWMCON2bits.FLTIEN = PWMCON3bits.FLTIEN = 0;	//使能故障中断
	PWMCON1bits.CLIEN = PWMCON2bits.CLIEN = PWMCON3bits.CLIEN = 1;		//禁止限流中断
	PWMCON1bits.TRGIEN = 1; //pwm1触发中断

	PTCON2 = 0X0002;  						//1:4 PRESCALER
	PTCONbits.SEIEN= 1;
	//PTCONbits.PTEN = 1;						// Enabling the PWM
        PTCONbits.PTEN = 0;		// disable PWM outputs

	TRIG1 = 600;

	IPC23bits.PWM1IP = 4;
	IFS5bits.PWM1IF = 0;
	IEC5bits.PWM1IE = 0;

	IPC23bits.PWM2IP = 4;					// PWM Interrupt Priority 4
	IFS5bits.PWM2IF = 0;
	IEC5bits.PWM2IE = 0;

	IPC24bits.PWM3IP = 4;					// PWM Interrupt Priority 4
	IFS6bits.PWM3IF = 0;
	IEC6bits.PWM3IE = 0;

}
/*********************************************************************
  Function:        void InitICandCN(void)

  Overview:        Configure Hall sensor inputs, one change notification and 
                   two input captures. on IC7 the actual capture value is used
                   for further period calculation

  Note:            None.
********************************************************************/

void InitIC(void)
{
	//Hall A -> IC1. Hall A is used for Speed measurement and commutation.
	//Hall B -> IC2. Hall B is only used for commutation.
	//Hall C -> IC3. Hall C is only used for commutation.

	IC1CON1 = 1;			// Init all 3 Hall Effect capture inputs:
	IC2CON1 = 1;			// Timer 3, every capture event, rise & fall edges
	IC3CON1 = 1;

        IPC0bits.IC1IP = 1;
        IPC1bits.IC2IP = 1;
        IPC9bits.IC3IP = 1;

	IFS0bits.IC1IF = 0;	// Clear interrupt flag
	IFS0bits.IC2IF = 0;
	IFS2bits.IC3IF = 0;

	IEC0bits.IC1IE = 0;	// Enable interrupt
	IEC0bits.IC2IE = 0;
	IEC2bits.IC3IE = 0;
}

/************************************************************************
Tmr3 is used to determine the rotor speed so it is set to count using Tcy/256

*************************************************************************/

void InitTMR3(void)
{
	T3CON = 0x0030;			// internal Tcy/256 clock
	TMR3 = 0;
	PR3 = 0xFFFF;
        IPC2bits.T3IP = 2;
}

/************************************************************************
Initialize the UART2 for BAUD = 9600, no parity, 1 stop

*************************************************************************/

void InitTMR1(void)
{
	// MIPs / (Scale * Hz interrupt)
	//  PR1=FCY MIP/(64 * 1000)  for 1000Hz interrupt
	
	T1CON = 0x8020;			// internal Tcy/64 clock
	TMR1 = 0;
	PR1 = T1PR1;	
	T1CONbits.TON = 0;		// turn on timer 1
        IPC0bits.T1IP = 3;      // Set Timer 1 Interrupt Priority Level
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 0;
	return;
}

//---------------------------------------------------------------------
// This is a generic 1ms delay routine to give a 1mS to 65.5 Seconds delay
// For N = 1 the delay is 1 mS, for N = 65535 the delay is 65,535 mS.
// Note that FCY is used in the computation.  Please make the necessary
// Changes(PLLx4 or PLLx8 etc) to compute the right FCY as in the define
// statement above.

void DelayNmSec(unsigned int N)
{
unsigned int j;
while(N--)
 	for(j=0;j < MILLISEC;j++);
}

/***********************************************************************************
 * Function: lockIO
 *
 * Preconditions: None.
 *
 * Overview: This executes the necessary process to set the IOLOCK bit to lock
 * I/O mapping from being modified.
 *
 * Input: None.
 *
 * Output: None.
 *
 *****************************************************************************/
void lockIO(){

asm volatile ("mov #OSCCON,w1 \n"
				"mov #0x46, w2 \n"
				"mov #0x57, w3 \n"
				"mov.b w2,[w1] \n"
				"mov.b w3,[w1] \n"
				"bset OSCCON, #6");
}

/*****************************************************************************
 * Function: unlockIO
 *
 * Preconditions: None.
 *
 * Overview: This executes the necessary process to clear the IOLOCK bit to
 * allow I/O mapping to be modified.
 *
 * Input: None.
 *
 * Output: None.
 *
 *****************************************************************************/
void unlockIO(){

asm volatile ("mov #OSCCON,w1 \n"
				"mov #0x46, w2 \n"
				"mov #0x57, w3 \n"
				"mov.b w2,[w1] \n"
				"mov.b w3,[w1] \n"
				"bclr OSCCON, #6");
}