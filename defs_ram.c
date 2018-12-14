/******************************************************************************************/
/*  FILE        :defs_ram.c                                                               */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

//#include <p33Fxxxx.h>
#include <p33Exxxx.h>
#include "defs_ram.h"
#include "pi.h"

/*************************************************************
	Low side driver table is as below.  In the StateLoTableClk
	and the StateLoTableAntiClk tables, the Low side driver is
	PWM while the high side driver is either on or off.
*************************************************************/
// ????????
unsigned int StateTableFwdPwm1[] =
{0xc301, 0xc301, 0xc341, 0xc341,0xc101, 0xc101, 0xc301, 0xc301};//a??U?

unsigned int StateTableFwdPwm2[] =
{0xc301, 0xc341, 0xc101, 0xc301,0xc301, 0xc341, 0xc101, 0xc301};//b??V?

unsigned int StateTableFwdPwm3[] =
{0xc301, 0xc101, 0xc301, 0xc101,0xc341, 0xc301, 0xc341, 0xc301};//C??W?
//       3H_2L   2H_1L   3H_1L  1H_3L   1H_2L   2H_3L

// ????????
unsigned int StateTableRevPwm1[] =
{0xc301, 0xc301, 0xc101, 0xc101,0xc341, 0xc341, 0xc301, 0xc301};//a??U?

unsigned int StateTableRevPwm2[] =
{0xc301, 0xc101, 0xc341, 0xc301,0xc301, 0xc101, 0xc341, 0xc301};//b??V?

unsigned int StateTableRevPwm3[] =
{0xc301, 0xc341, 0xc301, 0xc341,0xc101, 0xc301, 0xc101, 0xc301};//C??W?
//       2H_3L   1H_2L   1H_3L  3H_1L   2H_1L   3H_2L


struct MotorFlags Flags;

unsigned int HallValue;
unsigned int HallValue_Last;

unsigned int timer3value;
unsigned long timer3avg;
unsigned int timer3value_Last;
unsigned char FLAG_read_HALL_time;
int ActualSpeed;

unsigned int SET_SPEED;
int refSpeed;
unsigned int SPEED_open_loop_PDC;
unsigned int open_loop_time;
unsigned char flag_open_loop_time;
int SPEED_PDC;
int SPEED_PDC_offset;
int SPEED_PI_qOut;

unsigned int IBUS_value;
unsigned int IBUS_value_Last;
unsigned char FLAG_read_IBUS;
unsigned int  sum_IBUS_value;
unsigned int  avg_IBUS_value;
unsigned char IBUS_value_count;

UINT8 UART_RX_RT[50];
UINT8 UART1_DATA[50];
UINT8 UART_RX_idx;
UINT8 UART_RX_Size;
UINT8 FLAG_UART_R;
UINT16 UART_RX_check_SUM;
UINT16 BOOT_time;







UINT8 test_SPEED_PI_FLAG;



