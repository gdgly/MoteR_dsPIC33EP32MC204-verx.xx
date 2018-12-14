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
//// ????????
//unsigned int StateTableFwdPwm1[] =
//{0xc301, 0xc301, 0xc341, 0xc341,0xc101, 0xc101, 0xc301, 0xc301};//a??U?
//
//unsigned int StateTableFwdPwm2[] =
//{0xc301, 0xc341, 0xc101, 0xc301,0xc301, 0xc341, 0xc101, 0xc301};//b??V?
//
//unsigned int StateTableFwdPwm3[] =
//{0xc301, 0xc101, 0xc301, 0xc101,0xc341, 0xc301, 0xc341, 0xc301};//C??W?
////       3H_2L   2H_1L   3H_1L  1H_3L   1H_2L   2H_3L
//
//// ????????
//unsigned int StateTableRevPwm1[] =
//{0xc301, 0xc301, 0xc101, 0xc101,0xc341, 0xc341, 0xc301, 0xc301};//a??U?
//
//unsigned int StateTableRevPwm2[] =
//{0xc301, 0xc101, 0xc341, 0xc301,0xc301, 0xc101, 0xc341, 0xc301};//b??V?
//
//unsigned int StateTableRevPwm3[] =
//{0xc301, 0xc341, 0xc301, 0xc341,0xc101, 0xc301, 0xc101, 0xc301};//C??W?
////       2H_3L   1H_2L   1H_3L  3H_1L   2H_1L   3H_2L


////反向换相表 互补模式 
//#define off 0xc300 
//#define pwm 0xc100 
//#define UpoffDnon 0xc340 
//unsigned int StateTableRevPwm1[] = 
//{off, UpoffDnon, pwm      , off     , off      , UpoffDnon, pwm      , off};//a相即U相 
//
//unsigned int StateTableRevPwm2[] = 
//{off, off     , UpoffDnon ,UpoffDnon,pwm       , pwm      , off      , off};//b相即V相 
//
//unsigned int StateTableRevPwm3[] = 
//{off, pwm     , off       , pwm     , UpoffDnon, off      , UpoffDnon, off};//C相即W相
////    3H_1L     1H_2L       3H_2L     2H_3L      2H_1L      1H_3L

////以下为上桥PWM
//#define off 0xc301 
//#define pwm 0xc101 
//#define UpoffDnon 0xc341

//以下为下桥PWM
#define off 0xc301 
#define pwm 0xc201 
#define UpoffDnon 0xc381
// ????????
unsigned int StateTableFwdPwm1[] =
{off, off, UpoffDnon, UpoffDnon,pwm, pwm, off, off};//a??U?

unsigned int StateTableFwdPwm2[] =
{off, UpoffDnon, pwm, off,off, UpoffDnon, pwm, off};//b??V?

unsigned int StateTableFwdPwm3[] =
{off, pwm, off, pwm,UpoffDnon, off, UpoffDnon, off};//C??W?
//       3H_2L   2H_1L   3H_1L  1H_3L   1H_2L   2H_3L

// ????????
unsigned int StateTableRevPwm1[] =
{off, off, pwm, pwm,UpoffDnon, UpoffDnon, off, off};//a??U?

unsigned int StateTableRevPwm2[] =
{off, pwm, UpoffDnon, off,off, pwm, UpoffDnon, off};//b??V?

unsigned int StateTableRevPwm3[] =
{off, UpoffDnon, off, UpoffDnon,pwm, off, pwm, off};//C??W?
//       2H_3L   1H_2L   1H_3L  3H_1L   2H_1L   3H_2L



struct MotorFlags Flags;
unsigned int HallValue;
unsigned int HallValue_Last;
unsigned int timer3value;
unsigned long timer3avg;
unsigned int timer3value_Last;
unsigned char FLAG_read_HALL_time;
int ActualSpeed;

UINT32 Motor_place;
unsigned int SET_SPEED;
int refSpeed;
unsigned int SPEED_open_loop_PDC;
  /********************************************/
unsigned int open_loop_time;
UINT8 start_open_loop_step;
UINT8 start_close_loop_step;
UINT16 start_open_close_loop;
  /*******************************************/
unsigned char Flag_Motor_CloseLOOP;
unsigned char open_loop_inc;
unsigned int open_loop_inc_inc;
unsigned int Flag_CompareSpeed = 0; 

int SPEED_PDC;
int SPEED_PDC_out;
int SPEED_PDC_out_uart;
int SPEED_PDC_offset;
int SPEED_PI_qOut;

unsigned int IBUS_value;
unsigned int VBUS_value;
unsigned int IBUS_value_Last;
unsigned char FLAG_read_IBUS;


UINT8 UART_RX_RT[50];
UINT8 UART1_DATA[50];
UINT8 UART_RX_idx;
UINT8 UART_RX_Size;
UINT8 FLAG_UART_R;
UINT16 UART_RX_check_SUM;
UINT16 BOOT_time;
UINT8 Motor_MODE_B_data[def_MODE_B];
UINT16 SET_UP_SPEED_form_Uart;
UINT16 SET_DOWN_SPEED_form_Uart;
UINT16 UART_send_CMD;
UINT8 Motor_Origin_data[12];
UINT32 Motor_Origin_data_u32[3];

UINT8 TIME_down_limit;
UINT8 TIME_up_limit;

UINT8 Origin_mode_step;
UINT16 TIME_Origin_mode_learning;
UINT8 KEY_wired_value;
UINT8 KEY_wired_value_last;


UINT8 test_SPEED_PI_FLAG;

UINT8 TIME_SPEED_PDC_positive=0;
UINT8 TIME_SPEED_PDC_negative=0;
UINT8 Flag_DCInjection=0;

UINT16 TIME_Key_scan=0;

unsigned int avg_VBUS_value;
UINT8 TIME_chargeBootstraps=0;

