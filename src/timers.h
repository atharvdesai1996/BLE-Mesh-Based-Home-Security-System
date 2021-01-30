/**************************************************************************************************
 * @file      timers.h
 * @version   0.0.1
 * @brief     timers.h includes headers and function declarations needed in the file timers.h
 *
 * @author    Aaksha Jaywant, aaksha.jaywant@colorado.edu
 * @date      Sep 6, 2020
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor  David Sluiter
 *
 * @assignment ecen5823-assignment4-aakshajaywant
 * @due        Sep 25, 2020
 *
 * @resources  Utilized Silicon Labs' EMLIB peripheral libraries to implement functionality
 *
 * @copyright  All rights reserved. Distribution allowed only for the use of assignment grading.
 *       Use of code excerpts allowed at the discretion of author. Contact for permission.
 */



/*
 * timers.h
 *
 *  Created on: 06-Sep-2020
 *      Author: Aaksha Jaywant
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include <em_letimer.h>
#include <stdio.h>
//#include "main.h"
#include "irq.h"


//#define LedONTime 200
#define PRD 3000
void initialize_LETIMER0(uint32_t sFREQ);
void timerWaitUs(uint32_t us_wait);
uint32_t timerGetRunTimeMilliseconds(void);
#endif /* SRC_TIMERS_H_ */
