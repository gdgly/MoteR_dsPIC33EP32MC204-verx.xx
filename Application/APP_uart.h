/******************************************************************************************/
/*  FILE        :uart.h                                                                   */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

#ifndef APP_UART_H
#define APP_UART_H
#include "./Common/Typedefs/Typedefs.h"


#define def_MODE_B  50
extern UINT8 Motor_MODE_B_data[def_MODE_B];
extern UINT32 Motor_Origin_data_u32[3];
extern UINT16 test_SPEED_PI_FLAG;
extern UINT8 UART_KEY_wired_value;

void InitUART1(void);
void UART_Handler(void);
void UART_RX_decode(void);
void TEST_uart_speed_pi(void);
void UART_send_Motor(UINT16 d_COM,UINT8 d_addr,UINT8 d_length,UINT8 *d_data);


#endif /* APP_UART_H */
