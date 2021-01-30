/**************************************************************************************************
 * @file      irq.h
 * @version   0.0.1
 * @brief     irq.h includes headers and function declarations needed in the file irq.c
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
 *         		- system_efr32.h used for interrupt handler
 *				- https://www.silabs.com/content/usergenerated/asi/cloud/attachments/siliconlabs/en/community/mcu/32-bit/forum/jcr:content/content/primary/qna/letimer_for_low_ener-4b0X/LETimerSourceCode.c
 *
 * @copyright  All rights reserved. Distribution allowed only for the use of assignment grading.
 *       Use of code excerpts allowed at the discretion of author. Contact for permission.
 */

#ifndef SRC_IRQ_H_
#define SRC_IRQ_H_

#include "timers.h"
#include "em_core.h"
#include "scheduler.h"
#include "i2c.h"


#ifdef ENABLE_LOGGING
#define log(...) printf(__VA_ARGS__)
#else
#define log(...)
#endif
uint32_t TimestampCNT;

void LETIMER0_IRQHandler(void);
void I2C0_IRQHandler(void);


#endif /* SRC_IRQ_H_ */
