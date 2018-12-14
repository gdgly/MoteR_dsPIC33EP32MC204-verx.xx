/******************************************************************************************/
/*  FILE        :SensoredBLDC.h                                                           */
/*  DATE        :Mar, 2016                                                                */
/*  Programmer	:xiang 'R                                                                 */
/*  DESCRIPTION :                                                                         */
/******************************************************************************************/

extern unsigned int Read_Hall(void);
extern void Motor_Change_Phase(void);
extern void Motor_SPEED_Compute(void);
extern void RunMotor(void);
extern void StopMotor(void);
extern void runTestCode(void);
extern void adc_IBUS(void);