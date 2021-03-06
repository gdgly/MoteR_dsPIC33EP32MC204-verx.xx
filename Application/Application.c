/*********************************************************************************
* FileName: Application.c
* Description:
* This source file contains the definition of all the functions for the Application for Drive board.
**********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#include <p33Exxxx.h>
#include "Application.h"
#include "APP_uart.h"
#include "./Common/UserDefinition/Userdef.h"
#include "./Application/RampGenerator/RampGenerator.h"
#include "./Drivers/Timer/Timer.h"
#include "./Drivers/GPIO/GPIO.h"
#include "./Common/Extern/Extern.h"
#include "../Common/Delay/Delay.h"


WORD systemTick = 0;  
BOOL OvercurrentfaultTrigrd = FALSE; // indicating PWM fault due to overcurrent

UINT8 CMDStatus = 0;
BYTE MotorCountOver = 0;
WORD MotorStopCount;





                      /*****this is new add*********/  
  typedef union {
        UINT32	ul ;
	UINT8	u_char[4] ;
  }uni_ll;
  
#define SET_VBUS_PowerOFF_VAC  80   //??VAC
#define SET_VBUS_PowerOFF_VDC 113      //SET_VBUS_PowerOFF_VAC*1.414    //??VDC
#define SET_VBUS_PowerOFF     347      //(SET_VBUS_PowerOFF_VDC*2.2k/(82k+82k+56k+2.2k))/3.3*1024

#define MAXIMUM_WORKING_VOLTAGE_ac   178  //??VAC
#define MAXIMUM_WORKING_VOLTAGE_DC   252  //MAXIMUM_WORKING_VOLTAGE_ac*4.414  //??VDC
#define MAXIMUM_WORKING_VOLTAGE      774  //(MAXIMUM_WORKING_VOLTAGE_DC*2.2k/(82k+82k+56k+2.2k))/3.3*1024

  
struct MotorFlags M_Flags;
UINT8 KEY_wired_value;
UINT8 KEY_wired_value_last;
UINT16 TIME_Key_scan=0;
UINT16 SET_UP_SPEED_form_Uart;
UINT16 SET_DOWN_SPEED_form_Uart;
UINT8 Origin_mode_step;
unsigned int SET_SPEED;
UINT16 TIME_Origin_mode_learning;
UINT8 TIME_down_limit;
UINT8 TIME_up_limit;
UINT16 TIME_Origin_mode_down=0;
unsigned int avg_VBUS_value;

void Key_scan(void);
void SET_origin_mode(void);
void APP_Motor_MODE_B_data (void);
void Control_MotorForKey(void);
void UPlimitDOWNlimit(void);
void VBUS_PowerOFF_fun(void);
void adc_IBUSandVBUS(void);
/******************************************************************************
 * getSystemTick
 *
 * This function gets system tick count 
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/  
UINT32 getSystemTick(VOID)
{
	return systemTick; 
}

/******************************************************************************
 * application
 *
 * This function implements the task loop for the Application
 *
 * PARAMETER REQ: none
 *
 * RETURNS: none
 *
 * ERRNO: none
 ********************************************************************************/
VOID application(VOID)
{
#if 0    
        CMDStatus = (UINT8) ((In_OPEN<< 1) | (In_CLOSE));
        if((CMDStatus == 0x02) || (CMDStatus == 0x03))
        {
            Motor_ERR_overcurrent_or_igbtOverTemp=0;
        }
        
        if((CMDStatus == 0x00) && (!flags.motorRunning)&&(Motor_ERR_overcurrent_or_igbtOverTemp==0))
        {
            lockRelease;
            flags.RunDirection = CCW;            
            startMotor();
        }
        else if((CMDStatus == 0x01) && (!flags.motorRunning)&&(Motor_ERR_overcurrent_or_igbtOverTemp==0))
        {
            lockRelease; 
            flags.RunDirection = CW;
            startMotor();
        }
        else if((CMDStatus == 0x01) && (flags.motorRunning) && (flags.RunDirection == CCW))
        {
            if(MotorDecActive==0)
            {
                MotorDecActive = 1;
                MotorStopCount = 0;
            }
            else 
            {
                if((MotorStopCount >= 40)||(measuredSpeed_bak<200))
                {
                    stopMotor(); 
                    delayMs(100);
                    lockApply;                    
                }
                else if(MotorStopCount >= 60)
                {
                    flags.RunDirection = CW;
                    startMotor();
                    MotorStopCount = 0;
                }            
            } 
        }
        else if((CMDStatus == 0x00) && (flags.motorRunning) && (flags.RunDirection == CW))
        {
            if(MotorDecActive==0)
            {
                MotorDecActive = 1;
                MotorStopCount = 0;
            }
            else 
            {
                if((MotorStopCount >= 40)||(measuredSpeed_bak<200))
                {
                    stopMotor(); 
                    delayMs(100);
                    lockApply;                    
                }
                else if(MotorStopCount >= 60)
                {
                    flags.RunDirection = CCW;
                    startMotor();
                    MotorStopCount = 0;
                }            
            }
        }
        else if((CMDStatus == 0x02) && (flags.motorRunning))
        {
            if(MotorDecActive==0)
            {
                MotorDecActive = 1;
                MotorStopCount = 0;
            }
            else 
            {
                if((MotorStopCount >= 40)||(measuredSpeed_bak<200))
                {
                    stopMotor(); 
                    delayMs(100);
                    lockApply;                    
                }
            }
        }
        else if((CMDStatus == 0x03) && (flags.motorRunning))
        {
            if(MotorDecActive==0)
            {
                MotorDecActive = 1;
                MotorStopCount = 0;
            }
            else 
            {
                if((MotorStopCount >= 40)||(measuredSpeed_bak<200))
                {
                    stopMotor(); 
                    delayMs(100);
                    lockApply;                    
                }
            }
        }
#else 
      
        
     Key_scan();
     Control_MotorForKey();
     UPlimitDOWNlimit();
     SET_origin_mode();
     adc_IBUSandVBUS();
     
     //TEST_uart_speed_pi();
     
#endif
}

void __attribute__((interrupt, no_auto_psv)) _T9Interrupt (void)
{
    IFS3bits.T9IF = 0;
    if(TIME_Key_scan)TIME_Key_scan--;    
    MotorStopCount++;
    test_SPEED_PI_FLAG++;
    if(TIME_up_limit)TIME_up_limit--;
    if(TIME_down_limit)TIME_down_limit--; 
    if(TIME_Origin_mode_down)TIME_Origin_mode_down--;
    if(TIME_Origin_mode_learning)TIME_Origin_mode_learning--;
}







/*********************************************************************
  Function:        void adc_IBUS(void)

  Overview:        This is read IBUS from adc

  Note:            None.
********************************************************************/
void adc_IBUSandVBUS(void)
{
static unsigned int  sum_IBUS_value=0,sum_VBUS_value=0;
//static unsigned int  avg_IBUS_value=0,avg_VBUS_value=0;
static unsigned char IBUS_value_count=0,VBUS_value_count=0;
static UINT8 FLAG_powerOFF=0;

    if(FLAG_read_UBUS==1)
    {
        FLAG_read_UBUS=0;
//        sum_IBUS_value+=IBUS_value;
//        IBUS_value_count++;
//        if(IBUS_value_count>=8)
//        {
//            avg_IBUS_value=sum_IBUS_value>>3;
//            sum_IBUS_value=0;
//            IBUS_value_count=0;
//        }
        
        sum_VBUS_value+=UBUS;
        VBUS_value_count++;
        if(VBUS_value_count>=16)
        {
            avg_VBUS_value=sum_VBUS_value>>4;
            sum_VBUS_value=0;
            VBUS_value_count=0;
        }        
    }

//    if(avg_IBUS_value>SET_IBUS_Vavg_AD)   // ????30m??????6,????1.65V??????????????
//    {
//        StopMotor();
//        SET_SPEED=0;
//        lockApply;
//        avg_IBUS_value=0;
//        //Flags.flag_power_on=0;
//                           Flags.flag_open=0;
//                           Flags.flag_stop=0;
//                           Flags.flag_close=0;   
//        
//        Out_LED_PGD=1;
//    }
    if((avg_VBUS_value<SET_VBUS_PowerOFF)&&(M_Flags.flag_EEPROM_LOAD_OK==1)&&(FLAG_powerOFF==0))    // ?????????
    {
        FLAG_powerOFF=1;
        SET_SPEED=0;
        stopMotor(); 
        lockApply; 
        avg_VBUS_value=0;
                           M_Flags.flag_open=0;
                           M_Flags.flag_stop=0;
                           M_Flags.flag_close=0;   
        
        Out_LED_PGD=1;
        VBUS_PowerOFF_fun();
    }
}

/*********************************************************************
  Function:        void VBUS_PowerOFF_fun(void)

  Overview:

  Note:            None.
********************************************************************/
void VBUS_PowerOFF_fun(void)
{
    UINT8 d_number[6];
    uni_ll origin_l;
    UINT16 num_data;
    UINT16 UART_send_CMD;
    
    UART_send_CMD=0x8002;
    origin_l.ul=hallCounts;
    d_number[0]=origin_l.u_char[0];
    d_number[1]=origin_l.u_char[1];
    d_number[2]=origin_l.u_char[2];
    d_number[3]=origin_l.u_char[3];
    
    num_data=0;
    d_number[4]=num_data%256;
    d_number[5]=num_data>>8;
    
    UART_send_Motor(UART_send_CMD,0x01,0x06,d_number); 
}
/*********************************************************************
  Function:        void UPlimitDOWNlimit(void)

  Overview:

  Note:            None.
********************************************************************/
void UPlimitDOWNlimit(void)
{
if(Origin_mode_step==0)   //???????????????
{
    if(M_Flags.flag_down_limit==0){
        if(hallCounts>=Motor_Origin_data_u32[2]){
                           M_Flags.flag_open=0;
                           M_Flags.flag_stop=0;
                           M_Flags.flag_close=0;
            SET_SPEED=0;
            M_Flags.flag_down_limit=1;
            stopMotor(); 
            delayMs(20);
            lockApply; 
        }
        //else if((Motor_place>=(Motor_Origin_data_u32[2]-Motor_Origin_data_u32[2]/5))&&(Flags.flag_close==1)){
        else if((hallCounts>=(Motor_Origin_data_u32[2]-Motor_Origin_data_u32[2]*Motor_MODE_B_data[32]/100))&&(M_Flags.flag_close==1)){
            if(TIME_down_limit==0){
                TIME_down_limit=6; 
                if(SET_SPEED>=500)
                {
                    if(SET_DOWN_SPEED_form_Uart<=1000)SET_SPEED=SET_SPEED-2;  //100
                    else if(SET_DOWN_SPEED_form_Uart<=2000)SET_SPEED=SET_SPEED-50;
                    else SET_SPEED=SET_SPEED-120;
                }
                if(SET_SPEED<500)SET_SPEED=500;
            }
       }

    }
    else if(hallCounts<Motor_Origin_data_u32[2])M_Flags.flag_down_limit=0;

    if(M_Flags.flag_up_limit==0){
        if((hallCounts<=Motor_Origin_data_u32[1])&&(M_Flags.flag_power_on==0)){
                           M_Flags.flag_open=0;
                           M_Flags.flag_stop=0;
                           M_Flags.flag_close=0;
            SET_SPEED=0;
            M_Flags.flag_up_limit=1;
            if(M_Flags.flag_Origin_mode_down==1)TIME_Origin_mode_down=300;
            stopMotor(); 
            //delayMs(20);
            lockApply; 
        }
        //else if((Motor_place<=(Motor_Origin_data_u32[1]+Motor_Origin_data_u32[2]/5))&&(Flags.flag_open==1)){
        else if((hallCounts<=(Motor_Origin_data_u32[1]+Motor_Origin_data_u32[2]*Motor_MODE_B_data[31]/100))&&(M_Flags.flag_open==1)){
            if(TIME_up_limit==0){
                TIME_up_limit=6;  
                if(SET_SPEED>=500)
                {
                    if(SET_UP_SPEED_form_Uart<=1000)SET_SPEED=SET_SPEED-2; 
                    else if(SET_UP_SPEED_form_Uart<=2000)SET_SPEED=SET_SPEED-50;
                    else SET_SPEED=SET_SPEED-120;
                }
                if(SET_SPEED<500)SET_SPEED=500;
            }
        }
    }
    else if(hallCounts>Motor_Origin_data_u32[1])M_Flags.flag_up_limit=0;
    
    if((M_Flags.flag_Origin_mode_down==1)&&(TIME_Origin_mode_down==0)&&(M_Flags.flag_up_limit==1))
    {
        M_Flags.flag_Origin_mode_down=0;
        M_Flags.flag_open=0;
        M_Flags.flag_stop=0;
        M_Flags.flag_close=1;        
    }
}
else 
{
    if((TIME_Origin_mode_learning==0)&&(Origin_mode_step==4))
    {
               M_Flags.flag_EEPROM_LOAD_OK=1;
               Origin_mode_step=0;  //????????
               M_Flags.flag_open=1;
               M_Flags.flag_stop=0;
               M_Flags.flag_close=0;
               M_Flags.flag_origin=0;   
               M_Flags.flag_up_limit=0;
               M_Flags.flag_down_limit=1;
               
               M_Flags.flag_Origin_mode_down=1;
    }
}    
}
/*********************************************************************
  Function:        void SET_origin_mode(void)

  Overview:

  Note:            None.
********************************************************************/
void SET_origin_mode(void)
{
    UINT8 d_number[5];
    uni_ll origin_l;
    UINT16 UART_send_CMD;

    if(M_Flags.flag_origin==1)
    {
        M_Flags.flag_origin=0;
        Origin_mode_step++;

        UART_send_CMD=0x8001;
        d_number[0]=Origin_mode_step;
            M_Flags.flag_up_limit=0;
            M_Flags.flag_down_limit=0;
            if(Origin_mode_step==1) flags.motorRunning=0;
        if((Origin_mode_step>1)&&(Origin_mode_step<5))
        {
            if(Origin_mode_step==2)hallCounts=0;           
            Motor_Origin_data_u32[Origin_mode_step-2]=hallCounts;
            origin_l.ul=Motor_Origin_data_u32[Origin_mode_step-2];
            d_number[1]=origin_l.u_char[0];
            d_number[2]=origin_l.u_char[1];
            d_number[3]=origin_l.u_char[2];
            d_number[4]=origin_l.u_char[3];
        }
        UART_send_Motor(UART_send_CMD,0x01,0x05,d_number); 
        if(Origin_mode_step==4)
            TIME_Origin_mode_learning=320;
//            if(Origin_mode_step==4)
//            {
//                Flags.flag_EEPROM_LOAD_OK=1;
//                Origin_mode_step=0;  //????????
//                Flags.flag_open=1;  
//            }                      
    }
}
/*********************************************************************
  Function:        void Control_MotorForKey(void)

  Overview:        

  Note:            None.
********************************************************************/
void Control_MotorForKey(void)
{
             if((M_Flags.flag_open==1)&&(M_Flags.flag_up_limit==0))
             {
                 if(!flags.motorRunning)
                 { 
                    flags.RunDirection = M_Flags.flag_CW;
                    APP_Motor_MODE_B_data();
                    lockRelease; 
                    startMotor();
                 }
                 else
                 {
                     if(flags.RunDirection==M_Flags.flag_CW);
                     else {
                         SET_SPEED=0;
                         MotorDecActive = 1;
                         if(refSpeed<=200){
                             delayMs(20);
                             flags.RunDirection = M_Flags.flag_CW;
                             APP_Motor_MODE_B_data();
                             startMotor();
                         }
                     }
                 }
             }

             if((M_Flags.flag_close==1)&&(M_Flags.flag_down_limit==0))
             {
                 if(!flags.motorRunning)
                 {
                    flags.RunDirection = M_Flags.flag_CCW;
                    APP_Motor_MODE_B_data();
                    lockRelease; 
                    startMotor();
                 }
                 else
                 {
                     if(flags.RunDirection==M_Flags.flag_CCW);
                     else {
                         SET_SPEED=0;
                         MotorDecActive = 1;
                         if(refSpeed<=200){
                             delayMs(20);
                             flags.RunDirection = M_Flags.flag_CCW;
                             APP_Motor_MODE_B_data();
                             startMotor();
                         }
                     }
                 }
             }

             if((M_Flags.flag_stop==1) && (flags.motorRunning))
             {
                 SET_SPEED=0;
                 MotorDecActive = 1;
                 if(refSpeed<=200){
                    stopMotor(); 
                    delayMs(20);
                    lockApply; 
                 }
             }    
}

/*********************************************************************
  Function:        void Key_scan(void)

  Overview:        

  Note:            None.
********************************************************************/
void Key_scan(void)
{
    static UINT8 KEY_wired_value_old=0;
    
      if(In_OPEN==0)KEY_wired_value=KEY_wired_value|0x01;
      else KEY_wired_value=KEY_wired_value&0xfe;
      if(In_STOP==1)KEY_wired_value=KEY_wired_value|0x02;
      else KEY_wired_value=KEY_wired_value&0xfd;
      if(In_CLOSE==0)KEY_wired_value=KEY_wired_value|0x04;
      else KEY_wired_value=KEY_wired_value&0xfb;
      if(KEY_wired_value_last!=KEY_wired_value)
      {     
            KEY_wired_value_last=KEY_wired_value;
            if(KEY_wired_value==0x07)TIME_Key_scan=100;
            else TIME_Key_scan=5;
            return;
      }
      if(TIME_Key_scan) return;
      if((KEY_wired_value!=0x00)&&(KEY_wired_value_old!=KEY_wired_value))
      {
          KEY_wired_value_old=KEY_wired_value;
            switch(KEY_wired_value){             //????
                case 0x00:
    //                           if(Origin_mode_step!=0)   //?????????????OPEN/CLOSE
    //                           {
    //                               Flags.flag_open=0;
    //                               Flags.flag_stop=1;
    //                               Flags.flag_close=0;
    //                           }
                           break;
                case 0x01:
                               M_Flags.flag_open=1;
                               M_Flags.flag_stop=0;
                               M_Flags.flag_close=0;
                           break;
                case 0x02:
                               M_Flags.flag_open=0;
                               M_Flags.flag_stop=1;
                               M_Flags.flag_close=0;
                           break;
                case 0x04:
                               M_Flags.flag_open=0;
                               M_Flags.flag_stop=0;
                               M_Flags.flag_close=1;
                           break;
                case 0x07:
                               M_Flags.flag_open=0;
                               M_Flags.flag_stop=0;
                               M_Flags.flag_close=0;
                               M_Flags.flag_origin=1;
                           break;
                  default:
                            break;          
            }
            UART_KEY_wired_value=KEY_wired_value;
      }
}

/*********************************************************************
  Function:        void Control_MotorForKey(void)

  Overview:        

  Note:            None.
********************************************************************/
void APP_Motor_MODE_B_data (void)
{
    if(M_Flags.flag_open==1)
    {
           switch	( Motor_MODE_B_data[25] )
           {
                 case 1 :       //????
                       SET_UP_SPEED_form_Uart=1000;

                       break ;
                 case 2 :       //??2??
                       SET_UP_SPEED_form_Uart=2000;

                       break ;
                 case 3 :       //??3??
                       SET_UP_SPEED_form_Uart=2900;

                       break ;
                default:
                       break ;
           }
           SET_SPEED=SET_UP_SPEED_form_Uart;                 
    }
    else {
           switch	( Motor_MODE_B_data[26] )
           {
                 case 1 :       //??0.5??
                       SET_DOWN_SPEED_form_Uart=500;

                       break ;
                 case 2 :       //??0.75??
                       SET_DOWN_SPEED_form_Uart=750;

                       break ;
                 case 3 :       //????
                       SET_DOWN_SPEED_form_Uart=1000;

                       break ;
                 case 4 :       //??2??
                       SET_DOWN_SPEED_form_Uart=2000;

                       break ;
                 case 5 :       //??3??
                       SET_DOWN_SPEED_form_Uart=2900;

                       break ;
                default:
                       break ;
           }
           SET_SPEED=SET_DOWN_SPEED_form_Uart;                  

    }


    if(Origin_mode_step!=0)   //???????????????
    {
         //0.5??
                       SET_SPEED=750;    //500
    }
}

void BOOT_DELAY(void )
{
  UINT16  BOOT_time;
  
    for(BOOT_time=0;BOOT_time<6;BOOT_time++){
        delayMs(50);
        Out_LED_PGD=!Out_LED_PGD;
        ClrWdt();
    }
    Out_LED_PGD=0;
    M_Flags.flag_EEPROM_LOAD_OK=0;
}