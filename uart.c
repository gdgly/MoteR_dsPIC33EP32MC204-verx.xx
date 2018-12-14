/******************************************************************************************/
/*  FILE        :uart.c                                                                   */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

//#include <p33Fxxxx.h>
#include <p33Exxxx.h>
#include "defs_ram.h"
#include "Init.h"
#include "SensoredBLDC.h"
#include "PInew.H"

void Send_char(unsigned char ch){			// 发送字符
        while(U1STAbits.UTXBF);
	U1TXREG=ch;
}


void UART_ack(unsigned char ch)
{
  UINT16  sum;

	Send_char(UART1_DATA[0]);   //0xbb
        sum=UART1_DATA[0];
	Send_char(UART1_DATA[1]);   //0x00
        Send_char(UART1_DATA[2]);   //命令低字节
        sum= sum+UART1_DATA[2];
        Send_char(UART1_DATA[3]);  //命令高字节
        sum= sum+UART1_DATA[3];
        Send_char(UART1_DATA[4]);  //设备地址
        sum= sum+UART1_DATA[4];
        Send_char(ch);             //返信情报
        sum= sum+ch;
        Send_char(0x00);           //数据长度低字节
        Send_char(0x00);           //数据长度高字节
        Send_char(sum%256);        //sum_L
        Send_char(sum/256);        //sum_H
}


void UART_send_Motor(UINT16 d_COM,UINT8 d_addr,UINT8 d_length,UINT8 *d_data)
{
  UINT16  sum;
  UINT8 i,d_num;

	Send_char(0xbb);   //0xbb
        sum=0xbb;
	Send_char(0x00);   //0x00
        d_num=d_COM%256;
        Send_char(d_num);   //命令低字节
        sum= sum+d_num;
        d_num=d_COM/256;
        Send_char(d_num);  //命令高字节
        sum= sum+d_num;
        Send_char(d_addr);  //设备地址
        sum= sum+d_addr;
        Send_char(0x00);             //返信情报
        d_num=d_length%256;
        Send_char(d_num);           //数据长度低字节
        sum= sum+d_num;
        d_num=d_length/256;
        Send_char(d_num);           //数据长度高字节
        sum= sum+d_num;
        for(i=0;i<d_length;i++)
        {
          d_num=d_data[i];
          Send_char(d_num);   //数据 DATA
          sum= sum+d_num;
          //ClearWDT(); // Service the WDT
        }
        Send_char(sum%256);        //sum_L
        Send_char(sum/256);        //sum_H
}


void UART_RX_decode(void)
{
    unsigned char UART_DATA_j;
    UINT16 UART_RX_current_SUM;
           if(UART_RX_idx==2){
             if((UART_RX_RT[0]!=0xbb)||(UART_RX_RT[1]!=0x00)){
                  UART_RX_RT[0]=UART_RX_RT[1];
                  UART_RX_idx=1;
             }
             else UART_RX_check_SUM=0xbb+0x00;
           }
           else if(UART_RX_idx==8){
             UART_RX_Size=UART_RX_RT[7]<<8;
             UART_RX_Size=UART_RX_Size+ UART_RX_RT[6]+10;
           }
           else if(UART_RX_idx>9){
             if(UART_RX_idx==UART_RX_Size){
               UART_RX_check_SUM=UART_RX_check_SUM - UART_RX_RT[UART_RX_idx-1]- UART_RX_RT[UART_RX_idx-2];
               UART_RX_current_SUM=UART_RX_RT[UART_RX_idx-1]<<8;
               UART_RX_current_SUM=UART_RX_current_SUM+UART_RX_RT[UART_RX_idx-2];
                 if(UART_RX_current_SUM==UART_RX_check_SUM){
                     for(UART_DATA_j=0;UART_DATA_j<UART_RX_Size;UART_DATA_j++)UART1_DATA[UART_DATA_j]=UART_RX_RT[UART_DATA_j];
                     FLAG_UART_R=1;
                 }
                 else FLAG_UART_R=2;
               UART_RX_check_SUM=0;
               UART_RX_idx=0;
             }
           }
}

void UART_Handler(void)
{
    uni_i uart_x;
    uni_l uart_l;
    UINT8 uart_num,uart_i,d_x,num_data[6];

  if(FLAG_UART_R!=0){
    if(UART1_DATA[4]==2){         //电机mcu（设备addr:2）收到其它设备的呼叫
      if(FLAG_UART_R==1){   //接收数据完整，并校验
         UART_ack(0);
         uart_x.uc[0]=UART1_DATA[2];       //指令
         uart_x.uc[1]=UART1_DATA[3];
         switch(uart_x.ui){             //接收数据
            case 0x0101:         //无线控制指令  OPEN、STOP、CLOSE、登录+STOP
                        uart_num=UART1_DATA[8];
                        if(uart_num==0x01){
                           Flags.flag_open=1;
                           Flags.flag_stop=0;
                           Flags.flag_close=0;
                        }
                        else  if(uart_num==0x02){
                           Flags.flag_open=0;
                           Flags.flag_stop=1;
                           Flags.flag_close=0;
                        }
                        else  if(uart_num==0x04){
                           Flags.flag_open=0;
                           Flags.flag_stop=0;
                           Flags.flag_close=1;
                        }
                        else if(uart_num==0x07){
                           Flags.flag_open=0;
                           Flags.flag_stop=0;
                           Flags.flag_close=0;
                           Flags.flag_origin=1;
                        }
                        break;
             case 0x0102:  //如果进入MODE，即在操作按键，电机就停止
                        if((Flags.flag_open)||(Flags.flag_close)){
                           Flags.flag_open=0;
                           Flags.flag_stop=1;
                           Flags.flag_close=0;
                        }
                        break;
             case 0x0103:   //接收STM8上面的DIP SWITCH信息
                        uart_num=UART1_DATA[8];
                        if((uart_num==0)||(uart_num==3)){
                           Flags.flag_open=0;
                           Flags.flag_stop=1;
                           Flags.flag_close=0;
                        }
                        else if(uart_num==1){Flags.flag_CW=0;Flags.flag_CCW=1;}
                        else if(uart_num==2){Flags.flag_CW=1;Flags.flag_CCW=0;}
                        break;
             case 0x0201:  //将MODE所有参数从STM8上面copy下来
                        for(uart_i=0;uart_i<50;uart_i++)
                            Motor_MODE_B_data[uart_i]=UART1_DATA[uart_i+8];
                        break;
             case 0x0202:  //上电时，将原点、上限、下限位置从STM8取出
                        for(uart_i=0;uart_i<12;uart_i++)
                            Motor_Origin_data[uart_i]=UART1_DATA[uart_i+8];
                        for(uart_i=0;uart_i<3;uart_i++)
                        {
                            d_x=uart_i*4;
                            uart_l.u_char[0]=Motor_Origin_data[d_x];
                            uart_l.u_char[1]=Motor_Origin_data[d_x+1];
                            uart_l.u_char[2]=Motor_Origin_data[d_x+2];
                            uart_l.u_char[3]=Motor_Origin_data[d_x+3];
                            Motor_Origin_data_u32[uart_i]=uart_l.ul;
                        }
                        Flags.flag_EEPROM_LOAD_OK=1;
                        //if((Motor_Origin_data_u32[2]!=0)&&(Motor_Origin_data_u32[2]!=0xffffffff))Flags.flag_open=1;
                        break;
             case 0x0203:  //上电时，将电机位置信息从STM8取出
                        for(uart_i=0;uart_i<6;uart_i++)
                            num_data[uart_i]=UART1_DATA[uart_i+8];
                        uart_l.u_char[0]=num_data[0];
                        uart_l.u_char[1]=num_data[1];
                        uart_l.u_char[2]=num_data[2];
                        uart_l.u_char[3]=num_data[3];
                        Motor_place=uart_l.ul;     
                        
                        Flags.flag_EEPROM_LOAD_OK=1;
                        break;                        
                        
             case 0xEE01:     //test PI
                       /* Uart_PI_Speed_P=UART1_DATA[9]+(UART1_DATA[8]<<8);
                        Uart_PI_Speed_I=UART1_DATA[11]+(UART1_DATA[10]<<8);
                        Uart_PI_Speed_D=UART1_DATA[13]+(UART1_DATA[12]<<8);
                        Uart_PI_DCInjection_P=UART1_DATA[15]+(UART1_DATA[14]<<8);
                        Uart_PI_DCInjection_I=UART1_DATA[17]+(UART1_DATA[16]<<8);
                        Uart_PI_DCInjection_D=UART1_DATA[19]+(UART1_DATA[18]<<8);   */   

                        Uart_PI_DCInjection_P=UART1_DATA[9]+(UART1_DATA[8]<<8);
                        Uart_PI_DCInjection_I=UART1_DATA[11]+(UART1_DATA[10]<<8);
                        Uart_PI_DCInjection_D=UART1_DATA[13]+(UART1_DATA[12]<<8);
                        Uart_PI_DCInjection_MAX=UART1_DATA[15]+(UART1_DATA[14]<<8);
                        Uart_PI_DCInjection_MIN=UART1_DATA[17]+(UART1_DATA[16]<<8);
                        //=UART1_DATA[19]+(UART1_DATA[18]<<8);                
                        break;
              default:
                        break;
         }
      }
      else if(FLAG_UART_R==2)UART_ack(1);  //接收数据校验失败
    }
    else {         //电机mcu（设备addr:2）送信给其它设备后的反馈情况
      if(FLAG_UART_R==1)  //表示送信OK
      {
         uart_x.uc[0]=UART1_DATA[2];       //指令
         uart_x.uc[1]=UART1_DATA[3];
         switch(uart_x.ui){             
            case 0x8001:  
//                        if(Origin_mode_step==4)
//                        {
//                            Flags.flag_EEPROM_LOAD_OK=1;
//                            Origin_mode_step=0;  //退出原点设置模式
//                            Flags.flag_open=1;  
//                        }
                        break;
              default:
                        break;
         }                
      }
      else if(FLAG_UART_R==2);  //表示送信失败，需要处理
    }
    FLAG_UART_R=0;
  }
}








unsigned char hex_asc(unsigned char hex)
{
	unsigned char i;
	hex = hex & 0x0F;
	if (hex < 0x0A) i = hex | 0x30;
	else i = hex + 0x37;
	return i;
}
void TEST_uart_speed_pi(void)
{
    unsigned char  char_data;
    //unsigned char char0;
    //UINT8 d_xx[4];
    //uni_l d_num;    
    UINT16 DATA_TEST=0;

    if(test_SPEED_PI_FLAG>=2){
        test_SPEED_PI_FLAG=0;
        
        DATA_TEST=ActualSpeed;
        Send_char(0xA5);
        char_data=DATA_TEST>>8;
        Send_char(char_data);
        char_data=DATA_TEST%256;
        Send_char(char_data);
        DATA_TEST=SPEED_PDC;
        char_data=DATA_TEST>>8;
        Send_char(char_data);
        char_data=DATA_TEST%256;
        Send_char(char_data);        
        DATA_TEST=SPEED_PDC_out_uart;//SPEED_PDC_out;
        char_data=DATA_TEST>>8;
        Send_char(char_data);
        char_data=DATA_TEST%256;
        Send_char(char_data);   
        char_data=Flag_DCInjection*3;
        Send_char(char_data);
        DATA_TEST=VBUS_value;
        char_data=DATA_TEST>>8;
        Send_char(char_data);
        char_data=DATA_TEST%256;
        Send_char(char_data);
        DATA_TEST=avg_VBUS_value;//avg_VBUS_value;//IBUS_value;
        char_data=DATA_TEST>>8;
        Send_char(char_data);
        char_data=DATA_TEST%256;
        Send_char(char_data);        
        Send_char(0xAA);
          

    }
}
