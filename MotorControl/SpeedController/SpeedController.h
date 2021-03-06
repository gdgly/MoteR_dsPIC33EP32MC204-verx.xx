/********************************************************************************
* FileName: SpeedController.h
* Description:  
* This header file contains the decleration of all the attributes and 
* services for SpeedController.c file. It implements speed mode of
* motor operation
*********************************************************************************/

/****************************************************************************
 * Copyright 2017 Bunka MoteR.
 * This program is the property of the Bunka Shutters
 * Company, Inc.and it shall not be reproduced, distributed or used
 * without permission of an authorized company official.This is an
 * unpublished work subject to Trade Secret and Copyright protection.
*****************************************************************************/
#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H
#include <p33Exxxx.h>
#include "./MotorControl/PIController/pi.h"
#include "./Common/Typedefs/Typedefs.h"

#define PHASE_DECREMENT_FLAG    1
#define PHASE_INCREMENT_FLAG    2

#define HALLA_BIT	(PORTBbits.RB3)//(PORTBbits.RB1) /* HALLA port pin - RA8 RPI24 */
#define HALLB_BIT	(PORTBbits.RB2)//(PORTBbits.RB2) /* HALLB port pin - RC6 RP54 */
#define HALLC_BIT	(PORTBbits.RB1)//(PORTBbits.RB3) /* HALLC port pin - RF0 RPI96 */

EXTERN tPIParm speedPIparms;
/* This function initializes all the variables used by speed controller */
VOID intitSpeedController(VOID);

/* This function calculates phase value for given sector */
VOID calculatePhaseValue(WORD sectorNo);

SHORT getCurrentSectorNo(VOID);


#endif /* SPEED_CONTROLLER_H */
