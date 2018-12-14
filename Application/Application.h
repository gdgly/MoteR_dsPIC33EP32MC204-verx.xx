/********************************************************************************
* FileName: Application.h
* Description:  
* This header file contains the decleration of all the attributes and 
* services for Application.c file. It handles commands from the Control board acting as 
* an interface between communication module and ramp generator, Drive Fault checking, 
* System Counters, EEPROM and Anomaly history, updating Drive status and Fault registers. 
*********************************************************************************/

/****************************************************************************
 * Copyright 2014 Bunka Shutters.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright
 * protection.
*****************************************************************************/

/****************************************************************************
 *  Modification History
 *  
 *  Date                  Name          Comments 
 *  22/04/2014            iGate          Initial Creation                                                               
*****************************************************************************/
#ifndef APPLICATION_H
#define APPLICATION_H
#include "./Common/Typedefs/Typedefs.h"

EXTERN BOOL OvercurrentfaultTrigrd;

/* This function implements the task loop for the Drive Application */ 
VOID application(VOID); 

/* Method to get system tick counter */
UINT32 getSystemTick(VOID); 


#endif /* APPLICATION_H */

