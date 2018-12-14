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
#include "pi.h"
#include "PInew.h"
#include "DCInjection.h"
#include "APP_BX.h"

/*********************************************************************
  Function:        void Read_Hall(void)

  Overview:        Read Hall Value

  Note:            None.
********************************************************************/
unsigned int Read_Hall(void)
{
    unsigned int Read_Hall_Value;
    //Read_Hall_Value=(unsigned int)((PORTB >> 1) & 0x0007);	// Read halls
    Read_Hall_Value = HALLC_BIT + (HALLB_BIT << 1) + (HALLA_BIT << 2);
    return(Read_Hall_Value);
}

/*********************************************************************
  Function:        void Motor_Change_Phase(void)

  Overview:        Motor Change Phase

  Note:            None.
********************************************************************/
void Motor_Change_Phase(void)
{
    if(Flag_DCInjection!=1)
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
    }
    else 
        DCInjectionON();
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
#if  Phase_ICXorPWMInterrupt==0
	IEC0bits.IC1IE = 0;		// Enable interrupts on IC1
	IEC0bits.IC2IE = 1;		// Enable interrupts on IC2
	IEC2bits.IC3IE = 0;		// Enable interrupts on IC7        
#else
	IEC0bits.IC1IE = 1;		// Enable interrupts on IC1
	IEC0bits.IC2IE = 1;		// Enable interrupts on IC2
	IEC2bits.IC3IE = 1;		// Enable interrupts on IC7        
#endif        
	IEC5bits.PWM1IE = 1;	// Enable PWM interrupts

        Flags.RunMotor = 1;		// set flag
        refSpeed=600;                  //给定启动转速
        SPEED_PDC=150;                 //给定启动PWM占空比5%

        Flag_Motor_CloseLOOP=0;
        Flag_DCInjection=0;

        ActualSpeed=0;
        timer3value=MAX_PERIOD;
        timer3value_Last=MAX_PERIOD;
        timer3avg=MAX_PERIOD;

        //InitPI(&speed_PIparms,SPEED_PI_P,SPEED_PI_I,SPEED_PI_C,MAX_SPEED_PI,(-MAX_SPEED_PI),0);
        Speed_PID_init();
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
        SET_SPEED=0;
}
/*********************************************************************
  Function:        void APP_Motor_MODE_B_data (void)

  Overview:

  Note:            None.
********************************************************************/
/*
void APP_Motor_MODE_B_data (void)
{
    if(Flags.flag_open==1)
    {
           switch	( Motor_MODE_B_data[25] )
           {
                 case 1 :       //上升等速
                       SET_UP_SPEED_form_Uart=1000;
                       start_open_loop_step=50;
                       start_close_loop_step=7;
                       start_open_close_loop=100;
                       break ;
                 case 2 :       //上升2倍速
                       SET_UP_SPEED_form_Uart=2000;
                       start_open_loop_step=100;
                       start_close_loop_step=14;
                       start_open_close_loop=200;
                       break ;
                 case 3 :       //上升3倍速
                       SET_UP_SPEED_form_Uart=2900;
                       start_open_loop_step=150;
                       start_close_loop_step=20;
                       start_open_close_loop=300;
                       break ;
                default:
                       break ;
           }
           SET_SPEED=SET_UP_SPEED_form_Uart;
    }
    else {
           switch	( Motor_MODE_B_data[26] )
           {
                 case 1 :       //下降0.5倍速
                       SET_DOWN_SPEED_form_Uart=500;
                       start_open_loop_step=25;
                       start_close_loop_step=4;
                       start_open_close_loop=50;
                       break ;
                 case 2 :       //下降0.75倍速
                       SET_DOWN_SPEED_form_Uart=750;
                       start_open_loop_step=38;
                       start_close_loop_step=5;
                       start_open_close_loop=75;
                       break ;
                 case 3 :       //下降等速
                       SET_DOWN_SPEED_form_Uart=1000;
                       start_open_loop_step=50;
                       start_close_loop_step=7;
                       start_open_close_loop=100;
                       break ;
                 case 4 :       //下降2倍速
                       SET_DOWN_SPEED_form_Uart=2000;
                       start_open_loop_step=100;
                       start_close_loop_step=14;
                       start_open_close_loop=200;
                       break ;
                 case 5 :       //下降3倍速
                       SET_DOWN_SPEED_form_Uart=2900;
                       start_open_loop_step=150;
                       start_close_loop_step=20;
                       start_open_close_loop=300;
                       break ;
                default:
                       break ;
           }
           SET_SPEED=SET_DOWN_SPEED_form_Uart;
    }

    if(Origin_mode_step!=0)   //在设置原点、上限、下限时的转速
    {
         //0.5倍速
                       start_open_loop_step=100;
                       start_close_loop_step=14;
                       start_open_close_loop=200;
                       SET_SPEED=2000;
    }
}
 */


void APP_Motor_MODE_B_data (void)
{
    if(Flags.flag_open==1)
    {
           switch	( Motor_MODE_B_data[25] )
           {
                 case 1 :       //上升等速
                       SET_UP_SPEED_form_Uart=1000;

                       break ;
                 case 2 :       //上升2倍速
                       SET_UP_SPEED_form_Uart=2000;

                       break ;
                 case 3 :       //上升3倍速
                       SET_UP_SPEED_form_Uart=2900;

                       break ;
                default:
                       break ;
           }
#ifdef CLOSEDLOOP
           SET_SPEED=SET_UP_SPEED_form_Uart;           
#else
           SET_SPEED=SET_SPEED_ref;
#endif       
    }
    else {
           switch	( Motor_MODE_B_data[26] )
           {
                 case 1 :       //下降0.5倍速
                       SET_DOWN_SPEED_form_Uart=500;

                       break ;
                 case 2 :       //下降0.75倍速
                       SET_DOWN_SPEED_form_Uart=750;

                       break ;
                 case 3 :       //下降等速
                       SET_DOWN_SPEED_form_Uart=1000;

                       break ;
                 case 4 :       //下降2倍速
                       SET_DOWN_SPEED_form_Uart=2000;

                       break ;
                 case 5 :       //下降3倍速
                       SET_DOWN_SPEED_form_Uart=2900;

                       break ;
                default:
                       break ;
           }
#ifdef CLOSEDLOOP
           SET_SPEED=SET_DOWN_SPEED_form_Uart;           
#else
           SET_SPEED=SET_SPEED_ref;
#endif           

    }


    if(Origin_mode_step!=0)   //在设置原点、上限、下限时的转速
    {
         //0.5倍速
                       SET_SPEED=1000;    //500
    }

        if(SET_SPEED<=1000){open_loop_inc=1;open_loop_inc_inc=600;}
        else if(SET_SPEED<=2000) {open_loop_inc=1;open_loop_inc_inc=500;}
        else {open_loop_inc=2;open_loop_inc_inc=400;}
}

/*********************************************************************
  Function:        void runTestCode(void)

  Overview:        This is test code to run the motor

  Note:            None.

                                15k---->950
                                20k---->cw  Direction=0  650  3000rpm
 *                                                       1300 6000rpm
                                      ccw  Direction=1  750  3000rpm
********************************************************************/
void runTestCode(void)
{
//    if(Flags.flag_limit==0){
//        if(((Motor_place<=Motor_Origin_data_u32[1])&&(Flags.flag_power_on==0))||(Motor_place>=Motor_Origin_data_u32[2])){
//            Flags.flag_stop=1;
//            SET_SPEED=0;
//            Flags.flag_limit=1;
//            StopMotor();
//            DelayNmSec(20);
//            lockApply;
//        }
//    }
//  else if((Motor_place<Motor_Origin_data_u32[2])||(Motor_place>Motor_Origin_data_u32[1]))Flags.flag_limit=0;


    
    
if(Origin_mode_step==0)   //在设置原点、上限、下限时的转速
{
    if(Flags.flag_down_limit==0){
        if(Motor_place>=Motor_Origin_data_u32[2]){
                           Flags.flag_open=0;
                           Flags.flag_stop=0;
                           Flags.flag_close=0;
            SET_SPEED=0;
            Flags.flag_down_limit=1;
            StopMotor();
            DelayNmSec(20);
            lockApply;
        }
        //else if((Motor_place>=(Motor_Origin_data_u32[2]-Motor_Origin_data_u32[2]/5))&&(Flags.flag_close==1)){
        else if((Motor_place>=(Motor_Origin_data_u32[2]-Motor_Origin_data_u32[2]*Motor_MODE_B_data[32]/100))&&(Flags.flag_close==1)){
            if(TIME_down_limit==0){
                TIME_down_limit=100;
                SET_SPEED=SET_SPEED-100;
                //if(SET_SPEED<500)SET_SPEED=500;
                if(SET_SPEED<Motor_MODE_B_data[30]*100)SET_SPEED=Motor_MODE_B_data[30]*100;
            }
       }

    }
    else if(Motor_place<Motor_Origin_data_u32[2])Flags.flag_down_limit=0;

    if(Flags.flag_up_limit==0){
        if((Motor_place<=Motor_Origin_data_u32[1])&&(Flags.flag_power_on==0)){
                           Flags.flag_open=0;
                           Flags.flag_stop=0;
                           Flags.flag_close=0;
            SET_SPEED=0;
            Flags.flag_up_limit=1;
            StopMotor();
            //DelayNmSec(20);
            lockApply;
        }
        //else if((Motor_place<=(Motor_Origin_data_u32[1]+Motor_Origin_data_u32[2]/5))&&(Flags.flag_open==1)){
        else if((Motor_place<=(Motor_Origin_data_u32[1]+Motor_Origin_data_u32[2]*Motor_MODE_B_data[31]/100))&&(Flags.flag_open==1)){
            if(TIME_up_limit==0){
                TIME_up_limit=100;
                SET_SPEED=SET_SPEED-200;
                //if(SET_SPEED<500)SET_SPEED=500;
                if(SET_SPEED<Motor_MODE_B_data[29]*100)SET_SPEED=Motor_MODE_B_data[29]*100;
            }
        }
    }
    else if(Motor_place>Motor_Origin_data_u32[1])Flags.flag_up_limit=0;
}
else 
{
    if((TIME_Origin_mode_learning==0)&&(Origin_mode_step==4))
    {
               Flags.flag_EEPROM_LOAD_OK=1;
               Origin_mode_step=0;  //退出原点设置模式
               Flags.flag_open=1;
               Flags.flag_stop=0;
               Flags.flag_close=0;
               Flags.flag_origin=0;   
               Flags.flag_up_limit=0;
               Flags.flag_down_limit=1;
    }
}



#if defined(__Motor_debug__)
            if(SET_SPEED >= 500)
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


#else
                       
                        

             if((Flags.flag_open==1)&&(Flags.flag_up_limit==0))
             {
                 if(!Flags.RunMotor)
                 {
                    lockRelease;
                    DelayNmSec(20);
                    Flags.Direction = Flags.flag_CW;
                    APP_Motor_MODE_B_data();
                    RunMotor();
                 }
                 else
                 {
                     if(Flags.Direction==Flags.flag_CW);
                     else {
                         SET_SPEED=0;
                         if(refSpeed<=200){
                             DelayNmSec(20);
                             Flags.Direction = Flags.flag_CW;
                             APP_Motor_MODE_B_data();
                             RunMotor();
                         }
                     }
                 }
             }

             if((Flags.flag_close==1)&&(Flags.flag_down_limit==0))
             {
                 if(!Flags.RunMotor)
                 {
                    lockRelease;
                    DelayNmSec(20);
                    Flags.Direction = Flags.flag_CCW;
                    APP_Motor_MODE_B_data();
                    RunMotor();
                 }
                 else
                 {
                     if(Flags.Direction==Flags.flag_CCW);
                     else {
                         SET_SPEED=0;
                         if(refSpeed<=200){
                             DelayNmSec(20);
                             Flags.Direction = Flags.flag_CCW;
                             APP_Motor_MODE_B_data();
                             RunMotor();
                         }
                     }
                 }
             }

             if((Flags.flag_stop==1) && (Flags.RunMotor))
             {
                 SET_SPEED=0;
                 if(refSpeed<=200){
                     //DelayNmSec(100);
                     StopMotor();
                     DelayNmSec(20);
                     lockApply;
                 }
             }
#endif
}

/*********************************************************************
  Function:        void adc_IBUS(void)

  Overview:        This is read IBUS from adc

  Note:            None.
********************************************************************/
void adc_IBUSandVBUS(void)
{
static unsigned int  sum_IBUS_value=0,sum_VBUS_value=0;
static unsigned int  avg_IBUS_value=0;//,avg_VBUS_value=0;
static unsigned char IBUS_value_count=0,VBUS_value_count=0;
static UINT8 FLAG_powerOFF=0;

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
        
        sum_VBUS_value+=VBUS_value;
        VBUS_value_count++;
        if(VBUS_value_count>=16)
        {
            avg_VBUS_value=sum_VBUS_value>>4;
            sum_VBUS_value=0;
            VBUS_value_count=0;
        }        
    }
#if defined(__SOFT_Ver1__)
    if(avg_IBUS_value>=500)   // 取样电阻30m欧，放大倍数20，运放零点0.45V
    {
        StopMotor();
    }
#endif
#if defined(__SOFT_Ver2__) || defined(__SOFT_Ver3__)
    if(avg_IBUS_value>SET_IBUS_Vavg_AD)   // 取样电阻30m欧，放大倍数6,运放零点1.65V，反电动势正偏，负载电流反偏
    {
        StopMotor();
        SET_SPEED=0;
        lockApply;
        avg_IBUS_value=0;
        //Flags.flag_power_on=0;
                           Flags.flag_open=0;
                           Flags.flag_stop=0;
                           Flags.flag_close=0;   
        
        Out_LED_PGD=1;
    }
    if((avg_VBUS_value<SET_VBUS_PowerOFF)&&(Flags.flag_EEPROM_LOAD_OK==1)&&(FLAG_powerOFF==0))    // 断电时保存位置数据
    {
        FLAG_powerOFF=1;
        StopMotor();
        SET_SPEED=0;
        lockApply;
        avg_VBUS_value=0;
                           Flags.flag_open=0;
                           Flags.flag_stop=0;
                           Flags.flag_close=0;   
        
        Out_LED_PGD=1;
        VBUS_PowerOFF_fun();
    }
#endif
}

/*********************************************************************
  Function:        void Motor_Start_OpenLoop(void)

  Overview:        

  Note:            None.
********************************************************************/
void Motor_Start_OpenLoop(void)
{
    static unsigned int cnt100ms = 0; 
    unsigned int speed_start_error;
    
    if(((++cnt100ms >= Motor_MODE_B_data[27]*8)&&(ActualSpeed<800))||(ActualSpeed>=800))   
    {
        cnt100ms = 0;
        //Out_LED_PGD=!Out_LED_PGD;
    

       if(SET_SPEED >= 200)
        {
            if((ActualSpeed < SET_SPEED)&&(Flag_Motor_CloseLOOP==0))   
            {
                speed_start_error=(SET_SPEED-ActualSpeed)/open_loop_inc_inc;
                refSpeed = refSpeed+open_loop_inc+speed_start_error;
            }
            else Flag_Motor_CloseLOOP=1;
        }
        else
        {
            if(refSpeed > 200)
            {
                refSpeed -= 200;
            }
        }
 }
    
}