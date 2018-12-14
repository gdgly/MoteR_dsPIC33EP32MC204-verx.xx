/********************************************************************************
* FileName: Delay.h
* Description:  
* This header file contains the decleration of all the attributes and 
* services for Delay.c file. It implements delay module.
*********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#ifndef DELAY_H
#define DELAY_H
#include "./Common/Typedefs/Typedefs.h"

/* This function inserts 1us delay */
VOID delayUs(WORD delayCount);

/* This function inserts 1ms delay */
VOID delayMs(WORD delayCount);

#endif /* DELAY_H */
