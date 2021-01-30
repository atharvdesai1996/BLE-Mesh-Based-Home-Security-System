/**************************************************************************************************
 * @file      oscillators.h
 * @version   0.0.1
 * @brief     oscillators.h includes headers and function declarations needed in the file oscillators.h
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
 * @resources  Utilized Silicon Labs' EMLIB peripheral libraries to implement functionality.
 *
 * @copyright  All rights reserved. Distribution allowed only for the use of assignment grading.
 *       Use of code excerpts allowed at the discretion of author. Contact for permission.
 */


#ifndef SRC_OSCILLATORS_H_
#define SRC_OSCILLATORS_H_

#include <stdio.h>
#include <stdint.h>
#include "em_cmu.h"
#include <em_cmu.h>


uint32_t selectLXFO(void);
uint32_t selectULFRCO(void);
uint32_t clkEnableFUNC(void);


#endif /* SRC_OSCILLATORS_H_ */
