/******************************************************************************************/
/*  FILE        :Interrupts.c                                                             */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

//#include <p33Fxxxx.h>
#include <p33Exxxx.h>
#include "defs_ram.h"
#include "SensoredBLDC.h"
#include "uart.h"
#include "pi.h"
#include "PInew.h"
#include "DCInjection.h"

/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void)

Overview:

*********************************************************************/
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void)
{
    unsigned char dat;
    
    dat = U1RXREG;
           UART_RX_RT[UART_RX_idx++]=dat;  // Check preamble
           UART_RX_check_SUM=UART_RX_check_SUM+dat;
           UART_RX_decode();

    IFS0bits.U1RXIF = 0;
}
/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt (void)

Overview:		For Open loop, the ADC interrupt loads the PDCx 
				registers with thedemand pot value.  This is only 
				done when the motor is running.
				For Closed loop, the ADC interrupt saves into 
				DesiredSpeed the demand pot value.  This is only 
				done when the motor is running.

*********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _AD1Interrupt (void)
{

	//AD_SET_SPEED = ADC1BUF0 * POTMULT;	// value for speed control
        //if(AD_SET_SPEED<500)AD_SET_SPEED=500;

        IBUS_value_Last=IBUS_value;
#if defined(__SOFT_Ver1__)
        IBUS_value=ADC1BUF1;
        if((IBUS_value_Last>=1000)&&(IBUS_value>=1000))    // 取样电阻30m欧，放大倍数20，运放零点0.45V
            StopMotor();
#endif
#if defined(__SOFT_Ver2__)
        IBUS_value=ADC1BUF0;
        //if((IBUS_value_Last>SET_IBUS_Vpp_AD)&&(IBUS_value>SET_IBUS_Vpp_AD))    // 取样电阻30m欧，放大倍数6,运放零点1.65V，反电动势正偏，负载电流反偏
        //    StopMotor();
#endif
        FLAG_read_IBUS=1;
	// reset ADC interrupt flag
	IFS0bits.AD1IF = 0;
}

/**********************************************************************
PWM Interrupt Service Routine()
	- occurs every 50us
	- increments delay_counter ( for the DelayNmSec )
	- detects stalling
	- updates RTDM
	- is responsable for running the startup ramp
**********************************************************************/
void __attribute__((interrupt, no_auto_psv)) _PWM1Interrupt (void)
{
  	IFS5bits.PWM1IF = 0;	//clear PWM interrupt flag
	if(PWMCON1bits.FLTSTAT|PWMCON1bits.CLSTAT)
	{
 		StopMotor();
	}
#if Phase_ICXorPWMInterrupt==0
    HallValue = Read_Hall();	// Read halls
	if(HallValue!=HallValue_Last)Motor_Change_Phase();    
#endif

}

/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void)

PreCondition:
Overview:		This interrupt TIMER3.

********************************************************************/
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void)
{
    IFS0bits.T3IF = 0;
    timer3value = PR3;
}
/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt (void)

PreCondition:	The inputs of the hall effect sensors should have low pass
				filters. A simple RC network works.
 
Overview:		This interrupt represents Hall A ISR.
   				In Reverse, Hall reading == 3 or 4
   				In Forward, Hall reading == 2 or 5
   				and generates the next commutation sector.
				Hall A is used for Speed measurement
********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt (void)
{
	
	IFS0bits.IC1IF = 0;	// Clear interrupt flag
	HallValue = Read_Hall();	// Read halls	
        if(HallValue!=HallValue_Last)Motor_Change_Phase();       
		
}

/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt (void)

PreCondition:	The inputs of the hall effect sensors should have 
				low pass filters. A simple RC network works.
 
Overview:		This interrupt represents Hall B ISR.
   				Hall reading == 1 or 6
   				and generates the next commutation sector.
   				
********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt (void)
{
	IFS0bits.IC2IF = 0;	// Clear interrupt flag
#if Phase_ICXorPWMInterrupt==1
	HallValue = Read_Hall();	// Read halls
        if(HallValue!=HallValue_Last){    
            Motor_Change_Phase();            
#endif    
           T3CONbits.TON = 0;
           timer3value = TMR3;
           TMR3 = 0;
           T3CONbits.TON = 1;            
           FLAG_read_HALL_time=1;
           
            if(Flags.Direction == Flags.flag_CW){
                if(Motor_place>1)Motor_place--;
            }
            else Motor_place++;
           
#if Phase_ICXorPWMInterrupt==1           
        }
#endif    
}

/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _IC7Interrupt (void)

PreCondition:	The inputs of the hall effect sensors should have 
				low pass filters. A simple RC network works.
 
Overview:		This interrupt represents Hall C ISR.
   				and generates the next commutation sector.
   					
********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _IC3Interrupt (void)
{	
	IFS2bits.IC3IF = 0;	// Clear interrupt flag    
	HallValue = Read_Hall();	// Read halls
        if(HallValue!=HallValue_Last)Motor_Change_Phase();
}

/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void)

PreCondition:	None.
 
Overview:		This interrupt a 1ms interrupt and outputs a square
				wave toggling LED4.
   					
********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
  static unsigned int TIME_DCInjection=0;   
  static int PDC_DCInjection,Time_PDC_DCInjection;
    
    if(TIME_up_limit)TIME_up_limit--;
    if(TIME_down_limit)TIME_down_limit--;
    test_SPEED_PI_FLAG++;

    Motor_Start_OpenLoop();
    Motor_SPEED_Compute();

#ifndef CLOSEDLOOP
        SPEED_open_loop_PDC=refSpeed>>2;
        PDC1 = SPEED_open_loop_PDC;
	PDC2 = PDC1;
	PDC3 = PDC1;
#endif
#ifdef CLOSEDLOOP
         if(Flag_Motor_CloseLOOP==0)
         {
            SPEED_PDC=refSpeed/3;   //5
            if(SPEED_PDC>PWM_DutyCycle_MAX)SPEED_PDC=PWM_DutyCycle_MAX;
         }
        else
        {
//            speed_PIparms.qInRef = SET_SPEED;
//            speed_PIparms.qInMeas = ActualSpeed;
//            
//            CalcPI(&speed_PIparms);
//            SPEED_PI_qOut =  speed_PIparms.qOut;      //set PID output          
//
//            SPEED_PDC_offset =   __builtin_divsd((long)SPEED_PI_qOut*PWM_PTPER,MAX_SPEED_PI);
//            SPEED_PDC=SPEED_PDC+SPEED_PDC_offset;
             
                                   
	         PI_Speed.Ref = __builtin_divsd(((int64_t)SET_SPEED*(int64_t)32768),6000);     //计算成Q15，即Ref×32768/6000RPM; 
	         PI_Speed.Fdb = __builtin_divsd(((int64_t)ActualSpeed*(int64_t)32768),6000);          
             PICal(&PI_Speed);
             SPEED_PDC =PI_Speed.Out;
        }
    
    
    
        if(Flag_DCInjection==1)//&&(SPEED_PDC<0))
        {       
            Out_LED_PGD=1;
            if(ActualSpeed>SET_SPEED+20)
                PDC_DCInjection=PDC_DCInjection-(ActualSpeed-SET_SPEED)*1.8;  //1.5       
            SPEED_PDC_out=PWM_DutyCycle_MAX+PDC_DCInjection;
            if(SPEED_PDC_out<=0)SPEED_PDC_out=PWM_DutyCycle_MAX*0.12; //0.15 //0.12
        }
        if((SPEED_PDC<=0)&&(ActualSpeed>SET_SPEED+50)&&(Flags.Direction == Flags.flag_CW)&&(Motor_place<Motor_Origin_data_u32[2]/2))  
        {
            TIME_DCInjection++;           
            if(Flag_DCInjection==0)//&&(TIME_DCInjection>2))
            {
               SPEED_PDC_out=0; 
               SPEED_PDC=0;
               Flag_DCInjection=1;
               PDC_DCInjection=0;
               Time_PDC_DCInjection=0;   
            }
        }
        else if(Flag_DCInjection==1)//&&(ActualSpeed<SET_SPEED-200))
        {
                SPEED_PDC_out=0;
                Flag_DCInjection=0;
                Speed_PID_init();
                //Flag_Motor_CloseLOOP=0;
        }
        else if(Flag_DCInjection==0)
        {
            Out_LED_PGD=0;
            SPEED_PDC_out=SPEED_PDC; 
            TIME_DCInjection=0;
        }
            
        if(SPEED_PDC_out>PWM_DutyCycle_MAX)SPEED_PDC_out=PWM_DutyCycle_MAX;
    
        //PDC1 = SPEED_PDC; 
        PDC1 = SPEED_PDC_out;
        PDC2 = PDC1;
        PDC3 = PDC1;    
#endif								// in closed loop algorithm

	IFS0bits.T1IF = 0;
}
