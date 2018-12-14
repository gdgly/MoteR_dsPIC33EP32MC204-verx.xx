/****************************************Copyright (c)**************************************************
**                         Burnon
**---------------�ļ���Ϣ--------------------------------------------------------------------------------
**	��      ��  	��:	PI.c
**  ��  ��  ��  ��: by ChenPX 2017
**
**  ��  ��  ˵  ��:
**
********************************************************************************************************/
#include <p33Exxxx.h>
#include "PInew.H"
//#include "userdef.h"
#include "defs_ram.h"
#include "stdint.h"

#define Q15(Float_Value)	\
        ((Float_Value < 0.0) ? (int)(32768 * (Float_Value) - 0.5) \
        : (int)(32767 * (Float_Value) + 0.5))

//
//���Է���ʽKPx=150+ת��/2
#define KPL_Speed_PI Q15(0.033)   //(0.0299)//(0.0141) //	
#define KIL_Speed_PI Q15(0.012)   //Q15(0.01)//Q15(0.0185)  //(0.02)   //(0.0395)//	//ֵ�����ʱ���//0.05
#define KDL_Speed_PI Q15(0.009)   //(0.06)  

int Uart_KPL_Speed_PI=0;//32700;
int Uart_KIL_Speed_PI=0;//3300;
int Uart_KDL_Speed_PI=0;//100;
int Uart_KPL_Ibus_PI=0;
int Uart_KIL_Ibus_PI=0;
int Uart_KDL_Ibus_PI=0;

//TEST
#define KPL_Ibus_PI Q15(0.034)	//(0.0375)	150=0.045,50=0.015
#define KIL_Ibus_PI Q15(0.014)	//(0.0062)0.0032(0.0029)	     
#define KDL_Ibus_PI Q15(0.1)	//0.0711ֵԽ��,����ʱ��Խ��



#define KPR_Speed_PI Q15(0.033)   //(0.0299)//(0.0141) //	
#define KIR_Speed_PI Q15(0.012)   //Q15(0.01)//Q15(0.0185)  //(0.02)   //(0.0395)//	//ֵ�����ʱ���//0.05
#define KDR_Speed_PI Q15(0.009)   //(0.06)  

//TEST
#define KPR_Ibus_PI Q15(0.034)	//(0.0375)	150=0.045,50=0.015
#define KIR_Ibus_PI Q15(0.014)	//(0.0062)0.0032(0.0029)	     
#define KDR_Ibus_PI Q15(0.1)	//0.0711ֵԽ��,����ʱ��Խ��



#define KPL_Ipha_PI Q15(0.042)	//(0.04)
#define KIL_Ipha_PI Q15(0.0032)	//(0.0025)
#define KDL_Ipha_PI Q15(0.2)	//(0.05)

#define KPR_Ipha_PI Q15(0.042)	//(0.04)
#define KIR_Ipha_PI Q15(0.0032)	//(0.0025)
#define KDR_Ipha_PI Q15(0.2)	//(0.05)

PIDCTRL32 PI_SPL;
PIDCTRL32 PI_IDL;
PIDCTRL32 PI_IPL;

PIDCTRL32 PI_SPR;
PIDCTRL32 PI_IDR;
PIDCTRL32 PI_IPR;


unsigned int STATE;
/*********************************************************************************************************
** ��������: void PIInit(void)
** ��������:
**
** ����     ��:  ChenPX
** �ա�     ��:  2015
**********************************************************************************************************/
void PID_init(void)
{  
	PIRESet(&PI_SPL);
	PIRESet(&PI_IDL);
	PIRESet(&PI_IPL);

    //����
	PI_SPL.Ref = 0; //
	PI_SPL.Fdb = 0;

	PI_IPL.Ref = 32767; //100A=310.200A=620,150A=450
	PI_IPL.Fdb = 0;

	PI_IDL.Ref = 0; //33A
	PI_IDL.Fdb = 0;

//	PI_SPL.Kp  = KPL_Speed_PI;
//	PI_SPL.Kix = KIL_Speed_PI;
//	PI_SPL.Kc  = KDL_Speed_PI;

    if(SET_SPEED>=2000) 
    {
        Uart_KPL_Speed_PI=9000;
        Uart_KIL_Speed_PI=1500;
        Uart_KDL_Speed_PI=300;
    }
    else 
    {
        Uart_KPL_Speed_PI=5000;
        Uart_KIL_Speed_PI=1300;
        Uart_KDL_Speed_PI=200;
    }        
	PI_SPL.Kp  = Uart_KPL_Speed_PI;
	PI_SPL.Kix = Uart_KIL_Speed_PI;
	PI_SPL.Kc  = Uart_KDL_Speed_PI;    
	

	PI_IDL.Kp  = KPL_Ibus_PI;
	PI_IDL.Kix = KIL_Ibus_PI;
	PI_IDL.Kc  = KDL_Ibus_PI;

	PI_IPL.Kp  = KPL_Ipha_PI;
	PI_IPL.Kix = KIL_Ipha_PI;
	PI_IPL.Kc  = KDL_Ipha_PI;

	PI_IDL.OutMax = 0;
	PI_IDL.OutMin = -32760;

	PI_IPL.OutMax = 1200;
	PI_IPL.OutMin = 0;

	PI_SPL.OutMax = PWM_DutyCycle_MAX;//1200;
	PI_SPL.OutMin = 0;//-PWM_DutyCycle_MAX;//0;
    
    
    //�ҵ��
	PI_SPR.Ref = 0; //
	PI_SPR.Fdb = 0;

	PI_IPR.Ref = 32767; //100A=310.200A=620,150A=450
	PI_IPR.Fdb = 0;

	PI_IDR.Ref = 0; //33A
	PI_IDR.Fdb = 0;

	PI_SPR.Kp  = KPR_Speed_PI;
	PI_SPR.Kix = KIR_Speed_PI;
	PI_SPR.Kc  = KDR_Speed_PI;
	

	PI_IDR.Kp  = KPR_Ibus_PI;
	PI_IDR.Kix = KIR_Ibus_PI;
	PI_IDR.Kc  = KDR_Ibus_PI;

	PI_IPR.Kp  = KPR_Ipha_PI;
	PI_IPR.Kix = KIR_Ipha_PI;
	PI_IPR.Kc  = KDR_Ipha_PI;

	PI_IDR.OutMax = 0;
	PI_IDR.OutMin = -32760;

	PI_IPR.OutMax = 1200;
	PI_IPR.OutMin = 0;

	PI_SPR.OutMax = 1200;
	PI_SPR.OutMin = 0;
} 
/*********************************************************************************************************
** ��������: static void PIRESet(PIDCTRL32 *v)
** ��������: ��ʼ��PI�ṹ��
**
** ����     ��:  ChenPX
** �ա�     ��:
**********************************************************************************************************/
void PIRESet(PIDCTRL32 *v)
{
	v->Ref = 0;
	v->Fdb = 0;
	v->Err = 0;
	v->Kp  = 0;
	v->Kix  = 0;
	v->Kc  = 0;
	v->Up  = 0;
	v->Ui  = 0;
	v->OutPreSat = 0;
	v->OutMax = 0;
	v->OutMin = 0;
	v->Out    = 0;
	v->SatErr = 0;
	v->State = 0;
}
/*********************************************************************************************************
** ��������: void PICal(void)
** ��������: PI����������
**
** ����     ��:  ChenPX
** �ա�     ��:  2015
**********************************************************************************************************/
void PICal(PIDCTRL32 *v)
{    
    // �������
    v->Err = v->Ref - v->Fdb;
    if(v->Err>15000)
    {
        v->Err=15000;
    }
    else if(v->Err<-15000)
    {
        v->Err=-15000;
    }
    // ���������
    //v->Up = (int32_t)(((int64_t)v->Kp*(int64_t)v->Err)>>15);
	v->Up = __builtin_divsd(((int64_t)v->Kp*(int64_t)v->Err),32767);


    // ���������
    //v->Ui = v->Ui +(int32_t)(((int64_t)v->Kix*(int64_t)v->Up)>>15);
	v->Ui = v->Ui +__builtin_divsd(((int64_t)v->Kix*(int64_t)v->Up),32767);
	
    // ����׼�����
    v->OutPreSat = v->Up + v->Ui;

    // �����޷�
    if (v->OutPreSat > v->OutMax)
    {
    	v->Out =  v->OutMax;
    	//v->State = 1;
    }
    else if (v->OutPreSat < v->OutMin)
    {
    	v->Out =  v->OutMin;
    	//v->State = 2;
    }
    else
    {
      v->Out = v->OutPreSat;
      //v->State = 0;
    }
    // ���㱥�����
    v->SatErr = v->Out - v->OutPreSat;

    // ���»��ֱ���
    //v->Ui = v->Ui +(int32_t)(((int64_t)v->Kc*(int64_t)v->SatErr)>>15);
	v->Ui = v->Ui +__builtin_divsd(((int64_t)v->Kc*(int64_t)v->SatErr),32767);
	//STATE=PI_SP.State;
}