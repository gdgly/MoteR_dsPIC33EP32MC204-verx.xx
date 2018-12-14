/******************************************************************************************/
/*  FILE        :APP_BX.c                                                                 */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

//#include <p33Fxxxx.h>
#include <p33Exxxx.h>
#include "defs_ram.h"
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
        if((Origin_mode_step>1)&&(Origin_mode_step<5))
        {
            Motor_Origin_data_u32[Origin_mode_step-2]=Motor_place;
            origin_l.ul=Motor_Origin_data_u32[Origin_mode_step-2];
            d_number[1]=origin_l.u_char[0];
            d_number[2]=origin_l.u_char[1];
            d_number[3]=origin_l.u_char[2];
            d_number[4]=origin_l.u_char[3];
        }
        UART_send_Motor(UART_send_CMD,0x01,0x05,d_number);

    }
}

/*********************************************************************
  Function:        void Key_scan(void)

  Overview:        This is read IBUS from adc

  Note:            None.
********************************************************************/
void Key_scan(void)
{
      if(In_OPEN==0)KEY_wired_value=KEY_wired_value|0x01;
      else KEY_wired_value=KEY_wired_value&0xfe;
      if(In_STOP==1)KEY_wired_value=KEY_wired_value|0x02;
      else KEY_wired_value=KEY_wired_value&0xfd;
      if(In_CLOSE==0)KEY_wired_value=KEY_wired_value|0x04;
      else KEY_wired_value=KEY_wired_value&0xfb;
      if(KEY_wired_value_last!=KEY_wired_value){
        KEY_wired_value_last=KEY_wired_value;
        switch(KEY_wired_value){             //接收数据
            case 0x00:
                           if(Origin_mode_step!=0)
                           {
                               Flags.flag_open=0;
                               Flags.flag_stop=1;
                               Flags.flag_close=0;
                           }
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
