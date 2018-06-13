/********************************************************************
* Description: robotpos.h
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
* $Revision: 1.6 $
* $Author: cradek $
* $Date: 2007/07/14 21:43:24 $
********************************************************************/
#ifndef ROBOTPOS_H
#define ROBOTPOS_H

#include "posemath.h"		/* PmCartesian */

typedef struct RobotPose {
    PmCartesian tran;
    double a, b, c;
    double u, v, w;
} RobotPose;

#endif
