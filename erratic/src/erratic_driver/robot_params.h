// -*- mode:C; tab-width:2; c-basic-offset:2; indent-tabs-mode:1; -*-

/********************************************************************
 * Parameter struct for the robot
 *
 * Author: Joakim Arfvidsson
 * Copyright: Videre Design, 2006
 ********************************************************************/

/* this indicates compatibility of parameters struct */
#ifndef __ROBOT_PARAMS_H__
#define __ROBOT_PARAMS_H__

#define CURRENT_PARAMS_VERSION ((uint8_t)11)
#include <stdint.h>

struct parameters 
{
  char pName[20];
  char pType[20];
  char pSubtype[20];
  char pSN[10];

  uint8_t pVersion;
  uint8_t pSubVersion;

  uint8_t pBaudHost;
  uint8_t pBaudAux;

  uint16_t pRevCount;

  // watchdog timer
  uint16_t pWatchDog;		/* ms */

  // low battery warning
  uint16_t pLowBattery;		/* tenths of volts */

  uint16_t pStallVal;		/* PWM counts */
  uint16_t pStallCount;		/* how long we stall, in PERIOD ms increments */

  // PWM parameters
  uint16_t pPwmMax;		/* PWM counts; 0 will not work the motors */
  uint16_t pPwmWrap;		/* PWM timer wrap value */

  /* settings for onboard control */
  int16_t pRotVelMax;
  int16_t pTransVelMax;
  int16_t pRotAcc;
  int16_t pTransAcc;
  int16_t pRotDecel;
  int16_t pTransDecel;

  /* PID parameters */
  int16_t pTransKp;
  int16_t pTransKv;
  int16_t pTransKi;
  int16_t pRotKp;
  int16_t pRotKv;
  int16_t pRotKi;

  /* sonar parameters */
  int16_t pSonarConv;		/* converts from cycles to 256*mm */

  // added in rev 1.1
  // drift calibration, in ???
  int16_t pDriftBias;

#if 0
  // servo calibration
  int16_t pServoNeutral1;	/* neutral position, servo counts */
  int16_t pServoScale1;		/* counts per degree */
  int16_t pServoNeutral2;	/* neutral position, servo counts */
  int16_t pServoScale2;		/* counts per degree */
  int16_t pServoNeutral3;	/* neutral position, servo counts */
  int16_t pServoScale3;		/* counts per degree */
#endif
};


#endif