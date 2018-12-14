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

//      HallValue = Read_Hall();	// Read halls
//	if(HallValue!=HallValue_Last)Motor_Change_Phase();
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
	HallValue = Read_Hall();	// Read halls
        if(HallValue!=HallValue_Last){
            Motor_Change_Phase();
           T3CONbits.TON = 0;
           timer3value = TMR3;
           TMR3 = 0;
           T3CONbits.TON = 1;            
           FLAG_read_HALL_time=1;
        }
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
        if(HallValue!=HallValue_Last){
            Motor_Change_Phase();
            if(Flags.Direction == Flags.flag_CW){
                if(Motor_place>1)Motor_place--;
            }
            else Motor_place++;
        }
}

/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void)

PreCondition:	None.
 
Overview:		This interrupt a 1ms interrupt and outputs a square
				wave toggling LED4.
   					
********************************************************************/

/*
 void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
    static unsigned int cnt100ms = 0;

    open_loop_time++;
    if(open_loop_time>100)open_loop_time=100;

    if(++cnt100ms >= open_loop_time)
    {
        cnt100ms = 0;
        Out_LED_PGD=!Out_LED_PGD;
       if(SET_SPEED >= 100)
        {
            if(refSpeed < SET_SPEED)
            {
                //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                //说明：电机启动时，SET_SPEED转速的90%以下为开环，90%以上为闭环，开环时速度步进为150RPM，闭环时为20RPM，通过修改上面参数来完成加速时间。
                       //SET_SPEED=2900时，各参数如下：90%=300，速度步进为150RPM、20RPM
//                if(flag_open_loop_time==1)refSpeed += 150;
//                else refSpeed += 20;
//                flag_open_loop_time=1;
//                if(ActualSpeed>SET_SPEED-300)flag_open_loop_time=0;
                //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                if(flag_open_loop_time==1)refSpeed += start_open_loop_step;
                else refSpeed += start_close_loop_step;
                flag_open_loop_time=1;
                if(ActualSpeed>SET_SPEED-start_open_close_loop)flag_open_loop_time=0;
            }
            else refSpeed = SET_SPEED;
        }
        else if(refSpeed > 200) refSpeed -= 200;
    }

    Motor_SPEED_Compute();

#ifndef CLOSEDLOOP
        SPEED_open_loop_PDC=refSpeed>>2;
        PDC1 = SPEED_open_loop_PDC;
	PDC2 = PDC1;
	PDC3 = PDC1;    
#endif
#ifdef CLOSEDLOOP
        if(ActualSpeed<200)
          speed_PIparms.qInRef = ActualSpeed;
        else
          speed_PIparms.qInRef = refSpeed;
        speed_PIparms.qInMeas = ActualSpeed;

        CalcPI(&speed_PIparms);
        SPEED_PI_qOut =  speed_PIparms.qOut;      //set PID output

        SPEED_PDC_offset =   __builtin_divsd((long)SPEED_PI_qOut*1750,MAX_SPEED_PI);
        if(ActualSpeed<200)
           SPEED_PDC=refSpeed/5;
        else if((flag_open_loop_time==1)&&(SPEED_PDC_offset<0));
        else   SPEED_PDC=SPEED_PDC+SPEED_PDC_offset;


        if(SPEED_PDC<130)SPEED_PDC=130;
        else if(SPEED_PDC>1500)SPEED_PDC=1500;
        PDC1 = SPEED_PDC;
        PDC2 = PDC1;
        PDC3 = PDC1;

    test_SPEED_PI_FLAG++;
#endif								// in closed loop algorithm

	IFS0bits.T1IF = 0;
}
*/



void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
    static unsigned int cnt100ms = 0;
    static unsigned int CompareTime_ActualSpeed_SET_SPEED = 0;   
    unsigned int speed_start_error;

    if(TIME_up_limit)TIME_up_limit--;
    if(TIME_down_limit)TIME_down_limit--;

    if(((++cnt100ms >= Motor_MODE_B_data[27]*10)&&(ActualSpeed<800))||(ActualSpeed>=800))
    {
        cnt100ms = 0;
        //Out_LED_PGD=!Out_LED_PGD;
    

       if(SET_SPEED >= 200)
        {
            if((ActualSpeed < SET_SPEED)&&(flag_open_loop==0))
            {
                speed_start_error=(SET_SPEED-ActualSpeed)/open_loop_inc_inc;
                refSpeed = refSpeed+open_loop_inc+speed_start_error;
            }
            else flag_open_loop=1;
        }
        else
        {
            if(refSpeed > 200)
            {
                refSpeed -= 200;
            }
        }
 }
    
    Motor_SPEED_Compute();

#ifndef CLOSEDLOOP
        SPEED_open_loop_PDC=refSpeed>>2;
        PDC1 = SPEED_open_loop_PDC;
	PDC2 = PDC1;
	PDC3 = PDC1;
#endif
#ifdef CLOSEDLOOP
         if(flag_open_loop==0)
         {
            SPEED_PDC=refSpeed/3;   //5
            Flag_CompareSpeed=0;
         }
        else
        {
             if(ActualSpeed<SET_SPEED*4/5)
             {
                 CompareTime_ActualSpeed_SET_SPEED++;
                 if(CompareTime_ActualSpeed_SET_SPEED>=85)Flag_CompareSpeed=1;
             }
             else 
             {
                 CompareTime_ActualSpeed_SET_SPEED=0;
             }
             
             
            speed_PIparms.qInRef = SET_SPEED;
            speed_PIparms.qInMeas = ActualSpeed;

            CalcPI(&speed_PIparms);
            SPEED_PI_qOut =  speed_PIparms.qOut;      //set PID output

            SPEED_PDC_offset =   __builtin_divsd((long)SPEED_PI_qOut*1750,MAX_SPEED_PI);
            SPEED_PDC=SPEED_PDC+SPEED_PDC_offset;
        }


//        if(SPEED_PDC<130)SPEED_PDC=130;
//        else if(SPEED_PDC>1500)SPEED_PDC=1500;      
         
        if(SPEED_PDC<0)  
        {
            SPEED_PDC=-SPEED_PDC;
            if(Flag_DCInjection==0)
            {
               SPEED_PDC=0; 
               Flag_DCInjection=1;
            }
        }
        else if(SPEED_PDC>1500)SPEED_PDC=1500;
    
        PDC1 = SPEED_PDC;
        PDC2 = PDC1;
        PDC3 = PDC1;

    test_SPEED_PI_FLAG++;
#endif								// in closed loop algorithm

	IFS0bits.T1IF = 0;
}
