/******************************************************************************************/
/*  FILE        :defs_ram.h                                                               */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

//============================================
    #define	UINT8		unsigned char
    #define	INT8		char
    #define	UINT16		unsigned short int
    #define	INT16		short int
    #define	UINT32		unsigned long
    #define	INT32		long
    #define     NOP()           asm ("nop")
 //--------------------------------------------

//#define __SOFT_Ver1__       //hardware：（PCB 3.4.3）   PCB初次作成
//#define __SOFT_Ver2__       //hardware：（CY-1601 OP-0401）   PCB重新垒板，部分I/O有变动
#define __SOFT_Ver3__       //hardware：（CY-1601 OP Ver1.1 CQ:2017.07.18）   PCB重新垒板，部分I/O有变动
//#define __Motor_debug__     //电机上电后直接跑起，主要是测试电机部分电路是否正常
#define CLOSEDLOOP
#define Motor_Type   0       //电机制造商代码   1-->深圳东明电机     0-->文化BX提供
#define Phase_ICXorPWMInterrupt  0  //电机换相   1--》ICx换相       0--》PWM中断换相

//#define __RPI32_AD_SELCET__
#define __RPI32_UART_SELCET__

            //********************Fin16MHz_Fosc140MHz****************************************
            /*
            Using the Fosc = Fin * (M/(N1*N2)) =140M
                      Fcy  =	140M/2 = 70MIP
            */

            #define FOSC  140000000			// xtal = 16.0Mhz, 140.0Mhz after PLL
            #define FCY  FOSC/2
            #define FPWM 16000        //20000--》20KHz    16000--》16KHz
            #define PWM_ClockPrescaler   4    //PWM 4分频，如果需要改这个值的话，同时需要改 PTCON2
            #define PWM_PTPER  ((FOSC/PWM_ClockPrescaler)/FPWM)-1     //   1750--》20KHz   2188--》16KHz 
            #define PWM_DeadTime 56  //((FOSC/PWM_ClockPrescaler)*16)/10    //死区时间1.6us   ALTDTRx, DTRx = FOSC × Desired Dead TimePWM / Input Clock Prescaler
            #define PWM_DutyCycle_MAX   ((UINT32)(PWM_PTPER)*95)/100   //PWM_PTPER*95/100   最大占空比95%    

            #define BaudRate 454//(unsigned int)((FCY/(16*9600))-1)      //波特率：9600

            #define MILLISEC FCY/140000		// 1 mSec delay constant

            #define T1PR1 ((FCY/1000)/64)    //((FCY/1000)/64)

            /* Based on using the internal Fcy and Timer 3 prescaler of 256
             * Fcy/256 = 70M/256 = 273437.5 ticks/sec
             * or, 16406250 ticks = 1RPM
             * => RPM would be 16406250/T3ticks
             */
            #define SPEEDMULT	16406250
            #define POTMULT 4				// pot to speed ratio

           // Period Calculation
           // Period = (FCY / PDIV * 60) / (RPM * NO_POLEPAIRS )
           #define TIMER3_DIV   256
           #define POLEPAIRS		4		// number of pole pairs
           #define MAX_RPM         10000
           #define MIN_RPM         50//10//100
           #define MIN_PERIOD	((unsigned long)((FCY/TIMER3_DIV)*60)/((unsigned long)MAX_RPM*2*POLEPAIRS))
           #define MAX_PERIOD	((unsigned long)((FCY/TIMER3_DIV)*60)/((unsigned long)MIN_RPM*2*POLEPAIRS))
                   /*****MIN_PERIOD=342   (FCY=70MIPS,TIMER3_DIV=256,MAX_RPM=6000,POLEPAIRS=4)*******/
                   /*****MAX_PERIOD=41015 (FCY=70MIPS,TIMER3_DIV=256,MAX_RPM=50,POLEPAIRS=4)*******/

           /* for speed rpm calculation */
           #define SPEED_RPM_CALC      ((((unsigned long)FCY*60)/(TIMER3_DIV*2*POLEPAIRS)))
           #define SPEED_avg_pcs    8


           #define SPEED_PI_P  8000//8000//5000 6000
           #define SPEED_PI_I  5
           #define SPEED_PI_C  0

//           #define SPEED_PI_P  10000//5000 6000
//           #define SPEED_PI_I  0
//           #define SPEED_PI_C  0

           //since stack shuts down after 95% of PWM duty therefore limit PI max output to 90%
           #define MAX_SPEED_PI    31128   //95% of max value ie 32767
#ifdef CLOSEDLOOP
           #define SET_SPEED_ref   2900
#else
            #define SET_SPEED_ref   5000          //open loop
#endif

            #define T2PR1 ((FCY/1000)/256)

           //*********************************************************************************

#if defined(__SOFT_Ver2__) || defined(__SOFT_Ver3__)     //电流采样设置参数
           #define SET_IBUS_Vpp_protect  10    //单位A
           #define SET_IBUS_Vavg_protect  6    //单位A
           #define SET_IBUS_sample_R  30   //单位mΩ
           #define SET_IBUS_gain   6
           #define SET_IBUS_Vpp_AD   1010 //((1650+SET_IBUS_Vpp_protect*SET_IBUS_sample_R*SET_IBUS_gain)*1024/3300)
           #define SET_IBUS_Vavg_AD   850//750//850 //((1650+SET_IBUS_Vavg_protect*SET_IBUS_sample_R*SET_IBUS_gain)*1024/3300)

           #define SET_VBUS_PowerOFF_VAC  80   //单位VAC
           #define SET_VBUS_PowerOFF_VDC 113      //SET_VBUS_PowerOFF_VAC*1.414    //单位VDC
           #define SET_VBUS_PowerOFF     347      //(SET_VBUS_PowerOFF_VDC*2.2k/(82k+82k+56k+2.2k))/3.3*1024

           #define MAXIMUM_WORKING_VOLTAGE_ac   178  //单位VAC
           #define MAXIMUM_WORKING_VOLTAGE_DC   252  //MAXIMUM_WORKING_VOLTAGE_ac*4.414  //单位VDC
           #define MAXIMUM_WORKING_VOLTAGE      774  //(MAXIMUM_WORKING_VOLTAGE_DC*2.2k/(82k+82k+56k+2.2k))/3.3*1024
#endif


#if Motor_Type==0
        #define HALLA_BIT	(PORTBbits.RB1) /* HALLA port pin - RB1 */
        #define HALLB_BIT	(PORTBbits.RB2) /* HALLB port pin - RB2 */
        #define HALLC_BIT	(PORTBbits.RB3) /* HALLC port pin - RB3 */
#else
        #define HALLA_BIT	(PORTBbits.RB3) /* HALLA port pin - */
        #define HALLB_BIT	(PORTBbits.RB2) /* HALLB port pin - */
        #define HALLC_BIT	(PORTBbits.RB1) /* HALLC port pin - */
#endif


//========================================================================
#if defined(__SOFT_Ver1__)
    #define In_LOW_LIM_dir                TRISBbits.TRISB9
    #define In_STOP_dir                   TRISCbits.TRISC7
    #define In_OPEN_dir                   TRISCbits.TRISC8
    #define In_CLOSE_dir                  TRISCbits.TRISC9
    #define In_PHOTO_LED2_dir             TRISAbits.TRISA10
    #define In_PHOTO_LED1_dir             TRISAbits.TRISA7
    #define In_SENSOR_dir                 TRISAbits.TRISA1
    #define In_MSS1_dir                   TRISCbits.TRISC4
    #define In_MSS2_dir                   TRISCbits.TRISC5
    #define In_COLUMN_dir                 TRISBbits.TRISB7
    #define In_SCREEN_dir                 TRISCbits.TRISC0
    #define In_TEMP_PROTECT_dir           TRISCbits.TRISC3

    #define In_ADC_TMEP_dir               TRISBbits.TRISB0
    #define In_ADC_IBUS_dir                   TRISAbits.TRISA0
    #define In_HALL_U_dir                 TRISBbits.TRISB1
    #define In_HALL_V_dir                 TRISBbits.TRISB2
    #define In_HALL_W_dir                 TRISBbits.TRISB3
    #define In_FLT1_dir                   TRISBbits.TRISB8
    #define Out_PWM3H_dir                 TRISBbits.TRISB10
    #define Out_PWM3L_dir                 TRISBbits.TRISB11
    #define Out_PWM2H_dir                 TRISBbits.TRISB12
    #define Out_PWM2L_dir                 TRISBbits.TRISB13
    #define Out_PWM1H_dir                 TRISBbits.TRISB14
    #define Out_PWM1L_dir                 TRISBbits.TRISB15

    #define Out_ENABLE_BRAKE_dir          TRISAbits.TRISA9
    #define Out_RELAY_DOWN_dir            TRISBbits.TRISB4
    #define Out_RELAY_UP_dir              TRISAbits.TRISA8
    #define Out_RELAY_DOWN_LIM_dir        TRISCbits.TRISC1
    //#define Out_RELAY_UP_LIM_dir          TRISCbits.TRISC2
    #define Out_RELAY_UP_LIM_dir          TRISBbits.TRISB6

    #define Out_RELAY_LINKAGE_PGC_dir          TRISAbits.TRISA4
    #define Out_LED_PGD_dir               TRISBbits.TRISB5
    //========================================================
    #define In_LOW_LIM                    PORTBbits.RB9
    #define In_STOP                       PORTCbits.RC7
    #define In_OPEN                       PORTCbits.RC8
    #define In_CLOSE                      PORTCbits.RC9
    #define In_PHOTO_LED2                 PORTAbits.RA10
    #define In_PHOTO_LED1                 PORTAbits.RA7
    #define In_SENSOR                     PORTAbits.RA1
    #define In_MSS1                       PORTCbits.RC4
    #define In_MSS2                       PORTCbits.RC5
    #define In_COLUMN                     PORTBbits.RB7
    #define In_SCREEN                     PORTCbits.RC0
    #define In_TEMP_PROTECT               PORTCbits.RC3
    #define In_HALL_U                     PORTBbits.RB1
    #define In_HALL_V                     PORTBbits.RB2
    #define In_HALL_W                     PORTBbits.RB3

    #define Out_ENABLE_BRAKE              LATAbits.LATA9
    #define Out_RELAY_DOWN                LATBbits.LATB4
    #define Out_RELAY_UP                  LATAbits.LATA8
    #define Out_RELAY_DOWN_LIM            LATCbits.LATC1
    //#define Out_RELAY_UP_LIM              LATCbits.LATC2
    #define Out_RELAY_UP_LIM              LATBbits.LATB6

    #define Out_RELAY_LINKAGE_PGC              LATAbits.LATA4
    #define Out_LED_PGD                   LATBbits.LATB5
#endif
//========================================================================

//========================================================================
#if defined(__SOFT_Ver2__)
    #define In_LOW_LIM_dir                TRISAbits.TRISA8
    #define In_STOP_dir                   TRISAbits.TRISA9
    #define In_OPEN_dir                   TRISAbits.TRISA4
    #define In_CLOSE_dir                  TRISBbits.TRISB4
    #define In_PHOTO_LED2_dir             TRISCbits.TRISC3
    #define In_PHOTO_LED1_dir             TRISCbits.TRISC4
    #define In_SENSOR_dir                 TRISCbits.TRISC5
    #define In_MSS1_dir                   TRISCbits.TRISC0
    #define In_MSS2_dir                   TRISCbits.TRISC1
    #define In_COLUMN_dir                 TRISCbits.TRISC2
    #define In_SCREEN_dir                 TRISBbits.TRISB7
    #define In_TEMP_PROTECT_dir           TRISAbits.TRISA7

    #define In_ADC_TMEP_dir                   TRISAbits.TRISA0
    #define In_ADC_IBUS_dir                   TRISAbits.TRISA1
    #define In_HALL_U_dir                 TRISBbits.TRISB1
    #define In_HALL_V_dir                 TRISBbits.TRISB2
    #define In_HALL_W_dir                 TRISBbits.TRISB3
    #define In_FLT1_dir                   TRISBbits.TRISB8
    #define Out_PWM3H_dir                 TRISBbits.TRISB10
    #define Out_PWM3L_dir                 TRISBbits.TRISB11
    #define Out_PWM2H_dir                 TRISBbits.TRISB12
    #define Out_PWM2L_dir                 TRISBbits.TRISB13
    #define Out_PWM1H_dir                 TRISBbits.TRISB14
    #define Out_PWM1L_dir                 TRISBbits.TRISB15

    #define Out_ENABLE_BRAKE_dir          TRISAbits.TRISA10
    #define Out_RELAY_DOWN_dir            TRISCbits.TRISC9
    #define Out_RELAY_UP_dir              TRISCbits.TRISC8
    #define Out_RELAY_DOWN_LIM_dir        TRISCbits.TRISC7
    #define Out_RELAY_UP_LIM_dir          TRISBbits.TRISB9

    #define Out_RELAY_LINKAGE_PGC_dir          TRISBbits.TRISB6
    #define Out_LED_PGD_dir               TRISBbits.TRISB5
    //========================================================
    #define In_LOW_LIM                    PORTAbits.RA8
    #define In_STOP                       PORTAbits.RA9
    #define In_OPEN                       PORTAbits.RA4
    #define In_CLOSE                      PORTBbits.RB4
    #define In_PHOTO_LED2                 PORTCbits.RC3
    #define In_PHOTO_LED1                 PORTCbits.RC4
    #define In_SENSOR                     PORTCbits.RC5
    #define In_MSS1                       PORTCbits.RC0
    #define In_MSS2                       PORTCbits.RC1
    #define In_COLUMN                     PORTCbits.RC2
    #define In_SCREEN                     PORTBbits.RB7
    #define In_TEMP_PROTECT               PORTAbits.RA7
    #define In_HALL_U                     PORTBbits.RB1
    #define In_HALL_V                     PORTBbits.RB2
    #define In_HALL_W                     PORTBbits.RB3

    #define Out_ENABLE_BRAKE              LATAbits.LATA10
    #define Out_RELAY_DOWN                LATCbits.LATC9
    #define Out_RELAY_UP                  LATCbits.LATC8
    #define Out_RELAY_DOWN_LIM            LATCbits.LATC7
    #define Out_RELAY_UP_LIM              LATBbits.LATB9

    #define Out_RELAY_LINKAGE_PGC              LATBbits.LATB6
    #define Out_LED_PGD                   LATBbits.LATB5
#endif
//========================================================================

//========================================================================
#if defined(__SOFT_Ver3__)
    #define In_ADC_TMEP_dir                   TRISAbits.TRISA0      //相同的
    #define In_ADC_IBUS_dir                   TRISAbits.TRISA1
    #define In_HALL_U_dir                 TRISBbits.TRISB1
    #define In_HALL_V_dir                 TRISBbits.TRISB2
    #define In_HALL_W_dir                 TRISBbits.TRISB3
    #define In_FLT1_dir                   TRISBbits.TRISB8
    #define Out_PWM3H_dir                 TRISBbits.TRISB10
    #define Out_PWM3L_dir                 TRISBbits.TRISB11
    #define Out_PWM2H_dir                 TRISBbits.TRISB12
    #define Out_PWM2L_dir                 TRISBbits.TRISB13
    #define Out_PWM1H_dir                 TRISBbits.TRISB14
    #define Out_PWM1L_dir                 TRISBbits.TRISB15

    #define In_ADC_VBUS_dir               TRISCbits.TRISC0
    #define In_LOW_LIM_dir                TRISCbits.TRISC3       //不同的
    #define In_STOP_dir                   TRISAbits.TRISA8
    #define In_OPEN_dir                   TRISCbits.TRISC2
    #define In_CLOSE_dir                  TRISCbits.TRISC1   
//    #define In_PHOTO_LED2_dir             TRISCbits.TRISC3    // 该版本已经移到STM8L的显示上面去了
//    #define In_PHOTO_LED1_dir             TRISCbits.TRISC4
//    #define In_SENSOR_dir                 TRISCbits.TRISC5
    #define In_MSS1_dir                   TRISAbits.TRISA9
    #define In_MSS2_dir                   TRISAbits.TRISA4
    #define In_COLUMN_dir                 TRISBbits.TRISB4
//    #define In_SCREEN_dir                 TRISBbits.TRISB7  // 该版本已经删除了
    #define In_TEMP_PROTECT_dir           TRISAbits.TRISA10

    #define Out_DBR_CTRL_dir              TRISAbits.TRISA7
    #define Out_ENABLE_BRAKE_dir          TRISCbits.TRISC9
    #define Out_RELAY_DOWN_dir            TRISCbits.TRISC8
    #define Out_RELAY_UP_dir              TRISBbits.TRISB7
    #define Out_RELAY_DOWN_LIM_dir        TRISCbits.TRISC7
    #define Out_RELAY_UP_LIM_dir          TRISBbits.TRISB9

    #define Out_RELAY_LINKAGE_PGC_dir          TRISBbits.TRISB6
    #define Out_LED_PGD_dir               TRISBbits.TRISB5
    //========================================================
    #define In_HALL_U                     PORTBbits.RB1      //相同的
    #define In_HALL_V                     PORTBbits.RB2
    #define In_HALL_W                     PORTBbits.RB3

    #define In_LOW_LIM                    PORTCbits.RC3    //不同的
    #define In_STOP                       PORTAbits.RA8
    #define In_OPEN                       PORTCbits.RC2
    #define In_CLOSE                      PORTCbits.RC1
//    #define In_PHOTO_LED2                 PORTCbits.RC3
//    #define In_PHOTO_LED1                 PORTCbits.RC4
//    #define In_SENSOR                     PORTCbits.RC5
    #define In_MSS1                       PORTAbits.RA9
    #define In_MSS2                       PORTAbits.RA4
    #define In_COLUMN                     PORTBbits.RB4
//    #define In_SCREEN                     PORTBbits.RB7
    #define In_TEMP_PROTECT               PORTAbits.RA10

    #define Out_DBR_CTRL                  LATAbits.LATA7
    #define Out_ENABLE_BRAKE              LATCbits.LATC9
    #define Out_RELAY_DOWN                LATCbits.LATC8
    #define Out_RELAY_UP                  LATBbits.LATB7
    #define Out_RELAY_DOWN_LIM            LATCbits.LATC7
    #define Out_RELAY_UP_LIM              LATBbits.LATB9

    #define Out_RELAY_LINKAGE_PGC              LATBbits.LATB6
    #define Out_LED_PGD                   LATBbits.LATB5
#endif
//========================================================================

#define lockApply  (Out_ENABLE_BRAKE = 0 )
#define lockRelease (Out_ENABLE_BRAKE = 1)

  typedef union {
        UINT16	ui ;
	UINT8	uc[2] ;
  }uni_i;

  typedef union {
        UINT32	ul ;
	UINT8	u_char[4] ;
  }uni_l;

struct MotorFlags
{
unsigned RunMotor 	:1;
unsigned Direction	:1;
unsigned StartStop      :1;
unsigned flag_open      :1;
unsigned flag_close     :1;
unsigned flag_stop      :1;
unsigned flag_CW        :1;
unsigned flag_CCW       :1;
unsigned flag_origin    :1;
unsigned flag_power_on    :1;
unsigned flag_up_limit     :1;
unsigned flag_down_limit     :1;
unsigned flag_EEPROM_LOAD_OK     :1;
unsigned unused		:3;
};


extern unsigned int StateTableFwdPwm1[];
extern unsigned int StateTableFwdPwm2[];
extern unsigned int StateTableFwdPwm3[];
extern unsigned int StateTableRevPwm1[];
extern unsigned int StateTableRevPwm2[];
extern unsigned int StateTableRevPwm3[];


extern struct MotorFlags Flags;
extern unsigned int HallValue;
extern unsigned int HallValue_Last;
extern unsigned int timer3value;
extern unsigned int timer3value_Last;
extern unsigned long timer3avg;
extern unsigned char FLAG_read_HALL_time;
extern int ActualSpeed;

extern UINT32 Motor_place;
extern unsigned int SET_SPEED;
extern int refSpeed;
extern unsigned int SPEED_open_loop_PDC;
      /*************************************/
extern unsigned int open_loop_time;
extern UINT8 start_open_loop_step;
extern UINT8 start_close_loop_step;
extern UINT16 start_open_close_loop;
    /****************************************/

extern unsigned char Flag_Motor_CloseLOOP;
extern unsigned char open_loop_inc;
extern unsigned int open_loop_inc_inc;
extern unsigned int Flag_CompareSpeed;

extern int SPEED_PDC;
extern int SPEED_PDC_out;
extern int SPEED_PDC_offset;
extern int SPEED_PI_qOut;

extern unsigned int IBUS_value;
extern unsigned int VBUS_value;
extern unsigned int IBUS_value_Last;
extern unsigned char FLAG_read_IBUS;

extern UINT8 UART_RX_RT[50];
extern UINT8 UART1_DATA[50];
extern UINT8 UART_RX_idx;
extern UINT8 UART_RX_Size;
extern UINT8 FLAG_UART_R;
extern UINT16 UART_RX_check_SUM;
extern UINT16 BOOT_time;
#define def_MODE_B  50
extern UINT8 Motor_MODE_B_data[def_MODE_B];
extern UINT16 SET_UP_SPEED_form_Uart;
extern UINT16 SET_DOWN_SPEED_form_Uart;
extern UINT16 UART_send_CMD;
extern UINT8 Motor_Origin_data[12];
extern UINT32 Motor_Origin_data_u32[3];

extern UINT8 TIME_down_limit;
extern UINT8 TIME_up_limit;

extern UINT8 Origin_mode_step;
extern UINT16 TIME_Origin_mode_learning;
extern UINT8 KEY_wired_value;
extern UINT8 KEY_wired_value_last;



extern UINT8 test_SPEED_PI_FLAG;

extern UINT8 TIME_SPEED_PDC_positive;
extern UINT8 TIME_SPEED_PDC_negative;
extern UINT8 Flag_DCInjection;

extern UINT16 TIME_Key_scan;

extern unsigned int avg_VBUS_value;
