/******************************************************************************************/
/*  FILE        :SensoredBLDC.c                                                           */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

//#include <p33Fxxxx.h>
#include <p33Exxxx.h>
#include "defs_ram.h"
#include "Init.h"


/*********************************************************************
  Function:        void Read_Hall(void)

  Overview:        Read Hall Value

  Note:            None.
********************************************************************/
unsigned int Read_Hall(void)
{
    unsigned int Read_Hall_Value;
    Read_Hall_Value=(unsigned int)((PORTB >> 1) & 0x0007);	// Read halls
    return(Read_Hall_Value);
}

/*********************************************************************
  Function:        void Motor_Change_Phase(void)

  Overview:        Motor Change Phase

  Note:            None.
********************************************************************/
void Motor_Change_Phase(void)
{
	if (Flags.Direction)
	{
	        IOCON1 = StateTableFwdPwm1[HallValue];
		IOCON2 = StateTableFwdPwm2[HallValue];
		IOCON3 = StateTableFwdPwm3[HallValue];
		}
	else
	{
	        IOCON1 = StateTableRevPwm1[HallValue];
		IOCON2 = StateTableRevPwm2[HallValue];
		IOCON3 = StateTableRevPwm3[HallValue];
		}
        HallValue_Last = HallValue;
}
/*********************************************************************
  Function:        void Motor_SPEED_Compute(void)

  Overview:        Motor SPEED Compute

  Note:            None.
********************************************************************/
void Motor_SPEED_Compute(void)
{
    unsigned int period;

    if(FLAG_read_HALL_time)
    {
        period = timer3value;
        timer3value = 0;
        FLAG_read_HALL_time = 0;

        timer3avg=(timer3avg + timer3value_Last + period*2)>>2;
        timer3value_Last=period;
	if (timer3avg < MIN_PERIOD)
		timer3avg = MIN_PERIOD;
	else if (timer3avg > MAX_PERIOD)
		timer3avg = MAX_PERIOD;

	ActualSpeed = __builtin_divud(SPEED_RPM_CALC,timer3avg);
    }

}
/*********************************************************************
  Function:        void RunMotor(void)

  Overview:        Call this subroutine when first trying to run the motor and
                   the motor is previously stopped. RunMotor will charge
                   bootstrap caps, will initialize application variables, and
                   will enable all ISRs.

  Note:            None.
********************************************************************/
void RunMotor(void)
{
        PTCONbits.PTEN = 1;			// Enabling the PWM
	HallValue = Read_Hall();	// Read halls
        Motor_Change_Phase();

        TMR1 = 0;
        T1CONbits.TON = 1;                      // start tmr1
        TMR3 = 0;
        T3CONbits.TON = 1;			// start tmr3

	IFS0bits.T1IF = 0;		// Clear timer 1 flag
        IFS0bits.T3IF = 0;
	IFS0bits.IC1IF = 0;		// Clear interrupt flag
	IFS0bits.IC2IF = 0;		// Clear interrupt flag
	IFS2bits.IC3IF = 0;		// Clear interrupt flag
	IFS5bits.PWM1IF = 0;	        // Clear interrupt flag

	IEC0bits.T1IE = 1;		// Enable interrupts for timer 1
        IEC0bits.T3IE = 1;
	IEC0bits.IC1IE = 1;		// Enable interrupts on IC1
	IEC0bits.IC2IE = 1;		// Enable interrupts on IC2
	IEC2bits.IC3IE = 1;		// Enable interrupts on IC7
	IEC5bits.PWM1IE = 1;	// Enable PWM interrupts

        Flags.RunMotor = 1;		// set flag
}

/*********************************************************************
  Function:        void StopMotor(void)

  Overview:        Call this subroutine when first trying to run the motor and
                   the motor is previously stopped. RunMotor will charge
                   bootstrap caps, will initialize application variables, and
                   will enable all ISRs.

  Note:            None.
********************************************************************/
void StopMotor(void)
{
	PTCONbits.PTEN = 0;		// disable PWM outputs
  	IOCON1 = 0xc301;
	IOCON2 = 0xc301;
	IOCON3 = 0xc301;

        T1CONbits.TON = 0;             
        T3CONbits.TON = 0;
	IEC0bits.T1IE = 0;
        IEC0bits.T3IE = 0;
	IEC0bits.IC1IE = 0;
	IEC0bits.IC2IE = 0;
	IEC2bits.IC3IE = 0;
        IEC5bits.PWM1IE = 0;

	Flags.RunMotor = 0;			// reset run flag
        AD_SET_SPEED=0;
}

/*********************************************************************
  Function:        void runTestCode(void)

  Overview:        This is test code to run the motor

  Note:            None.
********************************************************************/
void runTestCode(void)
{
            if(AD_SET_SPEED >= 500)
            {
                Flags.StartStop = 1;
            }
            else
            {
                if(refSpeed <= 350)
                    Flags.StartStop = 0;
            }

            if ((Flags.StartStop == 1) && (!Flags.RunMotor) )
            {
                lockRelease;
                DelayNmSec(100);
                Flags.Direction = 0;
                RunMotor();
            }
            else if ((Flags.StartStop == 0) && (Flags.RunMotor))
            {
                StopMotor();
                DelayNmSec(100);
                lockApply;
            }
}

/*********************************************************************
  Function:        void adc_IBUS(void)

  Overview:        This is read IBUS from adc

  Note:            None.
********************************************************************/
void adc_IBUS(void)
{
    if(FLAG_read_IBUS==1)
    {
        FLAG_read_IBUS=0;
        sum_IBUS_value+=IBUS_value;
        IBUS_value_count++;
        if(IBUS_value_count>=8)
        {
            avg_IBUS_value=sum_IBUS_value>>3;
            sum_IBUS_value=0;
            IBUS_value_count=0;
        }
    }
    if(avg_IBUS_value>=500)   // 取样电阻30m欧，放大倍数20，运放零点0.45V
    {
        StopMotor();
    }
}