#ifndef PI_H
#define PI_H

/************************************************************************************************************************
* ?2011 Microchip Technology Inc.
*
* MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:
*
* You may use this software, and any derivatives
* created by any person or entity by or on your behalf, exclusively with Microchip? products.
* Microchip and its licensors retain all ownership and intellectual property rights in the
* accompanying software and in all derivatives hereto.  
*
* This software and any accompanying information is for suggestion only.
* It does not modify Microchip? standard warranty for its products.  You agree that you are
* solely responsible for testing the software and determining its suitability.
* Microchip has no obligation to modify, test, certify, or support the software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR
* A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP? PRODUCTS, COMBINATION WITH
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

//------------------  C API for PI routines ---------------------

typedef struct {
    long  qdSum;		//Integrator sum; 1.31 format
    int   qKp;			//Proportional Gain		 
    int   qKi;			//Integral Gain
    int   qKc;			//Anti-windup Gain
    int   qOutMax;		//PI Output maximum limit
    int   qOutMin;		//PI Output minimum limit
    int   qInRef; 		//Reference
    int   qInMeas;		//Measurement
    int   qOut;			//PI Output; 1.15 format
    } tPIParm;
extern tPIParm speed_PIparms;
/************************************************************************
InitPI - function to init the PI Controller
	tPIParam 	//see definition above
	Kp			//Proportional Gain		 
    Ki			//Integral Gain
    Kc			//Anti-windup Gain
    max 		//PI Output maximum limit
    min 	 	//PI Output minimum limit
	out			//PI initial output
************************************************************************/
void InitPI(tPIParm *pParm,int Kp,int Ki,int Kc,int max,int min,int out);

/* CalcPI - function to calculate the output of the PI */
void CalcPI( tPIParm *pParm);
#endif



