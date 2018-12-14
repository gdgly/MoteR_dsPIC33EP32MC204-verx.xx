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

void Send_char(unsigned char ch){			// �����ַ�
        while(U1STAbits.UTXBF);
	U1TXREG=ch;
}


void UART_ack(unsigned char ch)
{
  UINT16  sum;

	Send_char(UART1_DATA[0]);   //0xbb
        sum=UART1_DATA[0];
	Send_char(UART1_DATA[1]);   //0x00
        Send_char(UART1_DATA[2]);   //������ֽ�
        sum= sum+UART1_DATA[2];
        Send_char(UART1_DATA[3]);  //������ֽ�
        sum= sum+UART1_DATA[3];
        Send_char(UART1_DATA[4]);  //�豸��ַ
        sum= sum+UART1_DATA[4];
        Send_char(ch);             //�����鱨
        sum= sum+ch;
        Send_char(0x00);           //���ݳ��ȵ��ֽ�
        Send_char(0x00);           //���ݳ��ȸ��ֽ�
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
    UINT8 uart_num;

  if(FLAG_UART_R!=0){
    if(UART1_DATA[4]==2){         //���mcu���豸addr:2���յ������豸�ĺ���
      if(FLAG_UART_R==1){   //����������������У��
         UART_ack(0);
         uart_x.uc[0]=UART1_DATA[2];       //ָ��
         uart_x.uc[1]=UART1_DATA[3];
         switch(uart_x.ui){
            case 0x0101:
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
                        break;
           default:
                        break;
         }
      }
      else if(FLAG_UART_R==2)UART_ack(1);  //��������У��ʧ��
    }
    else {         //���mcu���豸addr:2�����Ÿ������豸��ķ������
      if(FLAG_UART_R==1);  //��ʾ����OK
      else if(FLAG_UART_R==2);  //��ʾ����ʧ�ܣ���Ҫ����
    }
    FLAG_UART_R=0;
  }
}
