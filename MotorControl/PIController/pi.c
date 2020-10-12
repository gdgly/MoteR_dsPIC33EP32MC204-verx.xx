/************************************************************************************************************************
* © 2011 Microchip Technology Inc.
*
* MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:
*
* You may use this software, and any derivatives
* created by any person or entity by or on your behalf, exclusively with Microchip’s products.
* Microchip and its licensors retain all ownership and intellectual property rights in the
* accompanying software and in all derivatives hereto.  
*
* This software and any accompanying information is for suggestion only.
* It does not modify Microchip’s standard warranty for its products.  You agree that you are
* solely responsible for testing the software and determining its suitability.
* Microchip has no obligation to modify, test, certify, or support the software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR
* A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP’S PRODUCTS, COMBINATION WITH
* ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH 
* OF STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST
* EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED
* THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.
***********************************************************************************************************************/

/*****************************************************************************/
//  File:       pi.c
//
//  Includes:
//      void InitPI(tPIParm *pParm,int Kp,int Ki,int Kc,int max,int min,int out)
//
/*****************************************************************************/

#include "pi.h"

/************************************************************************
InitPI - function to init the PI Controller
	tPIParam 	//see definition in pi.h
	Kp			//Proportional Gain		 
    Ki			//Integral Gain
    Kc			//Anti-windup Gain
    max 		//PI Output maximum limit
    min 	 	//PI Output minimum limit
	out			//PI initial output
************************************************************************/
void initPiNew(tPIParm *pParm,int Kp,int Ki,int Kc,int max,int min,int out)
{
    pParm->qdSum = 0;
    pParm->qKp = Kp;
    pParm->qKi = Ki;
    pParm->qKc = Kc;
    pParm->qOutMax = max;
    pParm->qOutMin = min;
    pParm->qOut = out;
    
}

/* CalcPI - function to calculate the output of the PI */
void calcPiNew( tPIParm *pParm)
{
    /*
;    Error  = Reference - Measurement
;    Up  = (Kp * Error)>>15;
;    Sum  = Sum + (Ki * Up)>>15;
;    U = Up + Sum;
;    if( U > Outmax )
;        Out = Outmax
;    else if( U < OutMin )
;        Out = OutMin
;    else
;        Out = U
;    Exc = Out - U
;    Sum = Sum + (Kc * Exc)>>15
*/
    int currentError;
    long Up,Ui,Uc,U;
    int outTemp;
    int Reference,Measurement;

    Reference = __builtin_divsd(((long)pParm->qInRef*(long)32768),10000);     //???Q15??Ref×32768/10000RPM; 
	Measurement = __builtin_divsd(((long)pParm->qInMeas*(long)32768),10000); 
    //Error  = Reference - Measurement
    currentError = Reference - Measurement;
    if(currentError>15000)
    {
        currentError=15000;
    }
    else if(currentError<-15000)
    {
        currentError=-15000;
    }
    
    //Up  = (Kp * Error)>>15;
    U = __builtin_mulss(currentError, pParm->qKp);
    Up = U>>15;
    //Sum  = Sum + (Ki * Up)>>15;
    U = __builtin_mulss(Up, pParm->qKi);
    Ui = U>>15;
    pParm->qdSum = pParm->qdSum + Ui;   
    
    //limit the output between the allowed limits
    //pParm->qOut is the PI output
    outTemp = (int)(pParm->qdSum + Up);
    if(outTemp >  pParm->qOutMax)
        pParm->qOut=  pParm->qOutMax;
    else if(outTemp < pParm->qOutMin)
        pParm->qOut =  pParm->qOutMin;
    else
        pParm->qOut = outTemp;
    
    
    //compute the difference between the limited and not limites output
    //currentError is used as a temporary variable
    currentError = pParm->qOut - outTemp;
    
    //Sum = Sum + (Kc * Exc)>>15
    U = __builtin_mulss(currentError,  pParm->qKc);
    Uc = U>>15;
    pParm->qdSum = pParm->qdSum + Uc;
}

