/******************************************************************************************/
/*  FILE        :APP_BX.c                                                                 */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

//#include <p33Fxxxx.h>
#include <p33Exxxx.h>
#include "defs_ram.h"
#include "SensoredBLDC.h"
#include "Init.h"
#include "pi.h"
#include "uart.h"

/*********************************************************************
  Function:        void SET_origin_mode(void)

  Overview:

  Note:            None.
********************************************************************/
void SET_origin_mode(void)
{
    UINT8 d_number[5];
    uni_l origin_l;

    if(Flags.flag_origin==1)
    {
        Flags.flag_origin=0;
        Origin_mode_step++;

        UART_send_CMD=0x8001;
        d_number[0]=Origin_mode_step;
            Flags.flag_up_limit=0;
            Flags.flag_down_limit=0;
            if(Origin_mode_step==1)Flags.RunMotor=0;
        if((Origin_mode_step>1)&&(Origin_mode_step<5))
        {
            if(Origin_mode_step==2)Motor_place=0;           
            Motor_Origin_data_u32[Origin_mode_step-2]=Motor_place;
            origin_l.ul=Motor_Origin_data_u32[Origin_mode_step-2];
            d_number[1]=origin_l.u_char[0];
            d_number[2]=origin_l.u_char[1];
            d_number[3]=origin_l.u_char[2];
            d_number[4]=origin_l.u_char[3];
        }
        UART_send_Motor(UART_send_CMD,0x01,0x05,d_number); 
        if(Origin_mode_step==4)
            TIME_Origin_mode_learning=3200;
//            if(Origin_mode_step==4)
//            {
//                Flags.flag_EEPROM_LOAD_OK=1;
//                Origin_mode_step=0;  //退出原点设置模式
//                Flags.flag_open=1;  
//            }                      
    }
    
    
                                        //以下是进入学习设置后，原点和上限自动学习
    if(Origin_mode_step==1)
    {
        if(
           (Flags.flag_PWMFLTorIBUS==1)//||
           //((ActualSpeed<=80)&&(TIME_Origin_mode_join==0)&&(Flags.Direction == Flags.flag_CW))  //MIN_RPM*7
           )        
        {
               Flags.flag_open=0;
               Flags.flag_stop=1;
               Flags.flag_close=0;   
               StopMotor();
               lockApply;
               
               Flags.flag_origin=1;
               TIME_Origin_mode_step=300;
        }
    }
    else if((Origin_mode_step==2)&&(TIME_Origin_mode_step==0))
    {
        if((Motor_place==0)&&(Flags.flag_close==0))
        {
               Flags.flag_open=0;
               Flags.flag_stop=0;
               Flags.flag_close=1;  
        }
        else if((Motor_place>=30)&&(Flags.RunMotor==1))   //100
        {
               Flags.flag_open=0;
               Flags.flag_stop=1;
               Flags.flag_close=0;   
               TIME_Origin_mode_step=500;            
        }
        else if((Motor_place>=30)&&(Flags.RunMotor==0))Flags.flag_origin=1;
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
    uni_l origin_l;
    UINT16 num_data;
    
    UART_send_CMD=0x8002;
    origin_l.ul=Motor_place;
    d_number[0]=origin_l.u_char[0];
    d_number[1]=origin_l.u_char[1];
    d_number[2]=origin_l.u_char[2];
    d_number[3]=origin_l.u_char[3];
    
    num_data=0;//Flags;
    d_number[4]=num_data%256;
    d_number[5]=num_data>>8;
    
    UART_send_Motor(UART_send_CMD,0x01,0x06,d_number); 
}

/*********************************************************************
  Function:        void Key_scan(void)

  Overview:        This is read IBUS from adc

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
            if(KEY_wired_value==0x07)TIME_Key_scan=1000;
            else TIME_Key_scan=50;
            return;
      }
      if(TIME_Key_scan) return;
      if((KEY_wired_value!=0x00)&&(KEY_wired_value_old!=KEY_wired_value))
      {
          KEY_wired_value_old=KEY_wired_value;
            switch(KEY_wired_value){             //接收数据
                case 0x00:
    //                           if(Origin_mode_step!=0)   //在设置原点时，需要一直按着OPEN/CLOSE
    //                           {
    //                               Flags.flag_open=0;
    //                               Flags.flag_stop=1;
    //                               Flags.flag_close=0;
    //                           }
                           break;
                case 0x01:
                               Flags.flag_open=1;
                               Flags.flag_stop=0;
                               Flags.flag_close=0;
                           break;
                case 0x02:
                               Flags.flag_open=0;
                               Flags.flag_stop=1;
                               Flags.flag_close=0;
                           break;
                case 0x04:
                               Flags.flag_open=0;
                               Flags.flag_stop=0;
                               Flags.flag_close=1;
                           break;
                case 0x07:
                               Flags.flag_open=0;
                               Flags.flag_stop=0;
                               Flags.flag_close=0;
                               Flags.flag_origin=1;
                           break;
                  default:
                            break;
            }
      }
}
