/******************************************************************************************/
/*  FILE        :uart.h                                                                   */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

void UART_Handler(void);
void UART_RX_decode(void);
void TEST_uart_speed_pi(void);
void UART_send_Motor(UINT16 d_COM,UINT8 d_addr,UINT8 d_length,UINT8 *d_data);
