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

int DesiredSpeed;
int SpeedError;
long SpeedIntegral = 0, SpeedIntegral_n_1 = 0, SpeedProportional = 0;
long DutyCycle = 0;
unsigned int Kps = 20000;					// Kp and Ks terms need to be adjusted
unsigned int Kis = 2000;					// as per the motor and load 


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
        IBUS_value=ADC1BUF1;
        if((IBUS_value_Last>=1000)&&(IBUS_value>=1000))    // 取样电阻30m欧，放大倍数20，运放零点0.45V
            StopMotor();
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
    static unsigned int cnt100ms = 0;

    //Out_RELAY_DOWN_LIM=!Out_RELAY_DOWN_LIM;

    if(++cnt100ms >= 100)
    {
        cnt100ms = 0;
        Out_LED_PGD=!Out_LED_PGD;
       if(AD_SET_SPEED >= 500)
        {
            if(refSpeed < AD_SET_SPEED)
            {
                refSpeed += 50;
                if(refSpeed >= AD_SET_SPEED)
                    refSpeed = AD_SET_SPEED;
            }
//            else if(refSpeed > AD_SET_SPEED){
//                if(refSpeed > 350)
//                {
//                    refSpeed -= 50;
//                }
//            }
        }
        else
        {
            if(refSpeed > 350)
            {
                refSpeed -= 50;
            }
        }
    }

    Motor_SPEED_Compute();

#ifndef CLOSEDLOOP
        PDC1 = refSpeed;
	PDC2 = PDC1;
	PDC3 = PDC1;    
#endif
#ifdef CLOSEDLOOP
	ActualSpeed = SPEEDMULT/timer3avg;
	SpeedError = DesiredSpeed - ActualSpeed;
	SpeedProportional = (int)(((long)Kps*(long)SpeedError) >> 15);
	SpeedIntegral = SpeedIntegral_n_1 + (int)(((long)Kis*(long)SpeedError) >> 15);
	
	if (SpeedIntegral < 0)
		SpeedIntegral = 0;
	else if (SpeedIntegral > 32767)
		SpeedIntegral = 32767;
	SpeedIntegral_n_1 = SpeedIntegral;
	DutyCycle = SpeedIntegral + SpeedProportional;
	if (DutyCycle < 0)
		DutyCycle = 0;
	else if (DutyCycle > 32767)
		DutyCycle = 32767;
	
	PDC1 = (int)(((long)(PTPER*2)*(long)DutyCycle) >> 15);
	PDC2 = PDC1;
	PDC3 = PDC1;
#endif								// in closed loop algorithm

	IFS0bits.T1IF = 0;
}

