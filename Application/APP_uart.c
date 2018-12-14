/******************************************************************************************/
/*  FILE        :uart.c                                                                   */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

#include <p33Exxxx.h>
#include "APP_uart.h"
#include "Application.h"
#include "./Common/UserDefinition/Userdef.h"
#include "./Application/RampGenerator/RampGenerator.h"
#include "./Drivers/Timer/Timer.h"
#include "./Drivers/GPIO/GPIO.h"
#include "./Common/Extern/Extern.h"
#include "../Common/Delay/Delay.h"



  typedef union {
        UINT16	ui ;
	UINT8	uc[2] ;
  }uni_i;
  typedef union {
        UINT32	ul ;
	UINT8	u_char[4] ;
  }uni_l;
  
UINT8 UART_RX_RT[50];
UINT8 UART1_DATA[50];
UINT8 UART_RX_idx;
UINT8 UART_RX_Size;
UINT8 FLAG_UART_R;
UINT16 UART_RX_check_SUM;
UINT8 Motor_Origin_data[12];
UINT32 Motor_Origin_data_u32[3];
UINT8 Motor_MODE_B_data[def_MODE_B];

UINT16 test_SPEED_PI_FLAG;
UINT8 UART_KEY_wired_value;
/*********************************************************************
  Function:        void InitUART1(void)

  Overview:        intializes the UART1

  Note:            None.
********************************************************************/
void InitUART1(void)
{
    TRISBbits.TRISB0=1;      //??????????UART RX???

    U1BRG = 454; //Set Baud rate 9600      ((FCY/(16*9600))-1)
    U1MODE = 0;
    U1STA = 0;

    IPC2bits.U1RXIP = 2; // Set priority level=1
    // Can be done in a single operation by assigning PC2SET = 0x0000000D
    IFS0bits.U1RXIF = 0; // Clear the timer interrupt status flag
    IEC0bits.U1RXIE = 1; // Enable timer interrupts

    U1MODEbits.UARTEN = 1; //Enable UART for 8-bit data
    //no parity, 1 Stop bit
    //U1STAbits.URXEN= 1; //Enable Transmit and Receive
    U1STAbits.UTXEN= 1;
}
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


void Send_char(unsigned char ch){			// ????
        while(U1STAbits.UTXBF);
	U1TXREG=ch;
}


void UART_ack(unsigned char ch)
{
  UINT16  sum;

	Send_char(UART1_DATA[0]);   //0xbb
        sum=UART1_DATA[0];
	Send_char(UART1_DATA[1]);   //0x00
        Send_char(UART1_DATA[2]);   //?????
        sum= sum+UART1_DATA[2];
        Send_char(UART1_DATA[3]);  //?????
        sum= sum+UART1_DATA[3];
        Send_char(UART1_DATA[4]);  //????
        sum= sum+UART1_DATA[4];
        Send_char(ch);             //????
        sum= sum+ch;
        Send_char(0x00);           //???????
        Send_char(0x00);           //???????
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
        Send_char(d_num);   //?????
        sum= sum+d_num;
        d_num=d_COM/256;
        Send_char(d_num);  //?????
        sum= sum+d_num;
        Send_char(d_addr);  //????
        sum= sum+d_addr;
        Send_char(0x00);             //????
        d_num=d_length%256;
        Send_char(d_num);           //???????
        sum= sum+d_num;
        d_num=d_length/256;
        Send_char(d_num);           //???????
        sum= sum+d_num;
        for(i=0;i<d_length;i++)
        {
          d_num=d_data[i];
          Send_char(d_num);   //?? DATA
          sum= sum+d_num;
          //ClearWDT(); // Service the WDT
        }
        Send_char(sum%256);        //sum_L
        Send_char(sum/256);        //sum_H
}

void UART_Handler(void)
{
    uni_i uart_x;
    uni_l uart_l;
    UINT8 uart_num,uart_i,d_x,num_data[6];

  if(FLAG_UART_R!=0){
    if(UART1_DATA[4]==2){         //??mcu???addr:2??????????
      if(FLAG_UART_R==1){   //??????????
         UART_ack(0);
         uart_x.uc[0]=UART1_DATA[2];       //??
         uart_x.uc[1]=UART1_DATA[3];
         switch(uart_x.ui){             //????
            case 0x0101:         //??????  OPEN?STOP?CLOSE???+STOP
                        uart_num=UART1_DATA[8];
                        if(uart_num==0x01){
                           M_Flags.flag_open=1;
                           M_Flags.flag_stop=0;
                           M_Flags.flag_close=0;
                        }
                        else  if(uart_num==0x02){
                           M_Flags.flag_open=0;
                           M_Flags.flag_stop=1;
                           M_Flags.flag_close=0;
                        }
                        else  if(uart_num==0x04){
                           M_Flags.flag_open=0;
                           M_Flags.flag_stop=0;
                           M_Flags.flag_close=1;
                        }
                        else if(uart_num==0x07){
                           M_Flags.flag_open=0;
                           M_Flags.flag_stop=0;
                           M_Flags.flag_close=0;
                           M_Flags.flag_origin=1;
                        }
                        break;
             case 0x0102:  //????MODE?????????????
                        if((M_Flags.flag_open)||(M_Flags.flag_close)){
                           M_Flags.flag_open=0;
                           M_Flags.flag_stop=1;
                           M_Flags.flag_close=0;
                        }
                        break;
             case 0x0103:   //??STM8???DIP SWITCH??
                        uart_num=UART1_DATA[8];
                        if((uart_num==0)||(uart_num==3)){
                           M_Flags.flag_open=0;
                           M_Flags.flag_stop=1;
                           M_Flags.flag_close=0;
                        }
                        else if(uart_num==1){M_Flags.flag_CW=0;M_Flags.flag_CCW=1;}
                        else if(uart_num==2){M_Flags.flag_CW=1;M_Flags.flag_CCW=0;}
                        break;
             case 0x0201:  //?MODE?????STM8??copy??
                        for(uart_i=0;uart_i<50;uart_i++)
                            Motor_MODE_B_data[uart_i]=UART1_DATA[uart_i+8];
                        break;
             case 0x0202:  //????????????????STM8??
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
                        M_Flags.flag_EEPROM_LOAD_OK=1;
                        //if((Motor_Origin_data_u32[2]!=0)&&(Motor_Origin_data_u32[2]!=0xffffffff))Flags.flag_open=1;
                        break;
             case 0x0203:  //????????????STM8??
                        for(uart_i=0;uart_i<6;uart_i++)
                            num_data[uart_i]=UART1_DATA[uart_i+8];
                        uart_l.u_char[0]=num_data[0];
                        uart_l.u_char[1]=num_data[1];
                        uart_l.u_char[2]=num_data[2];
                        uart_l.u_char[3]=num_data[3];
                        hallCounts=uart_l.ul;     
                        
                        M_Flags.flag_EEPROM_LOAD_OK=1;
                        break;                        
                        
             case 0xEE01:     //test PI
                       /* Uart_PI_Speed_P=UART1_DATA[9]+(UART1_DATA[8]<<8);
                        Uart_PI_Speed_I=UART1_DATA[11]+(UART1_DATA[10]<<8);
                        Uart_PI_Speed_D=UART1_DATA[13]+(UART1_DATA[12]<<8);
                        Uart_PI_DCInjection_P=UART1_DATA[15]+(UART1_DATA[14]<<8);
                        Uart_PI_DCInjection_I=UART1_DATA[17]+(UART1_DATA[16]<<8);
                        Uart_PI_DCInjection_D=UART1_DATA[19]+(UART1_DATA[18]<<8);   */   

                        /*Uart_PI_DCInjection_P=UART1_DATA[9]+(UART1_DATA[8]<<8);
                        Uart_PI_DCInjection_I=UART1_DATA[11]+(UART1_DATA[10]<<8);
                        Uart_PI_DCInjection_D=UART1_DATA[13]+(UART1_DATA[12]<<8);
                        Uart_PI_DCInjection_MAX=UART1_DATA[15]+(UART1_DATA[14]<<8);
                        Uart_PI_DCInjection_MIN=UART1_DATA[17]+(UART1_DATA[16]<<8);*/
                        //=UART1_DATA[19]+(UART1_DATA[18]<<8);                
                        break;
              default:
                        break;
         }
      }
      else if(FLAG_UART_R==2)UART_ack(1);  //????????
    }
    else {         //??mcu???addr:2??????????????
      if(FLAG_UART_R==1)  //????OK
      {
         uart_x.uc[0]=UART1_DATA[2];       //??
         uart_x.uc[1]=UART1_DATA[3];
         switch(uart_x.ui){             
            case 0x8001:  
//                        if(Origin_mode_step==4)
//                        {
//                            Flags.flag_EEPROM_LOAD_OK=1;
//                            Origin_mode_step=0;  //????????
//                            Flags.flag_open=1;  
//                        }
                        break;
              default:
                        break;
         }                
      }
      else if(FLAG_UART_R==2);  //???????????
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
        
        DATA_TEST=measuredSpeed;      //DATA1  bit16
        Send_char(0xA5);
        char_data=DATA_TEST>>8;
        Send_char(char_data);
        char_data=DATA_TEST%256;
        Send_char(char_data);
        DATA_TEST=hallCounts;          //DATA2  bit16
        if(DATA_TEST>65535)DATA_TEST=65535;
        char_data=DATA_TEST>>8;
        Send_char(char_data);
        char_data=DATA_TEST%256;
        Send_char(char_data);        
        DATA_TEST=UBUS;                     //DATA3  bit16
        char_data=DATA_TEST>>8;
        Send_char(char_data);
        char_data=DATA_TEST%256;
        Send_char(char_data);   
        if(M_Flags.flag_CW==0)
        char_data=0;                        //DATA4  bit8
        else char_data=1;
        Send_char(char_data);
        DATA_TEST=0;                      //DATA5 bit16
        char_data=DATA_TEST>>8;
        Send_char(char_data);
        char_data=DATA_TEST%256;
        Send_char(char_data);
        DATA_TEST=0;                      //DATA6 bit16
        char_data=DATA_TEST>>8;
        Send_char(char_data);
        char_data=DATA_TEST%256;
        Send_char(char_data);        
        Send_char(0xAA);
          

    }
}
