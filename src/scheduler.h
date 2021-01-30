/**************************************************************************************************
 * @file      scheduler.h
 * @version   0.0.1
 * @brief     scheduler.h includes headers and function declarations needed in the file scheduler.c
 *
 * @author    Aaksha Jaywant, aaksha.jaywant@colorado.edu
 * @date      Sep 14, 2020
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor  David Sluiter
 *
 * @assignment ecen5823-assignment4-aakshajaywant
 * @due        Sep 25, 2020
 *
 * @resources  Utilized Silicon Labs' EMLIB peripheral libraries to implement functionality.
 * 				1) https://www.geeksforgeeks.org/find-position-of-the-only-set-bit/
 *				2) sleep.h to use sleep functions
 *				3) Lec6 Scheduler Guidance
 * @copyright  All rights reserved. Distribution allowed only for the use of assignment grading.
 *       Use of code excerpts allowed at the discretion of author. Contact for permission.
 */


#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include "timers.h"
#include "i2c.h"
#define INCLUDE_LOG_DEBUG 1
//#include "log.h"
#include "em_core.h"
#include "sleep.h"
//#include "ble.h"
#include "native_gecko.h"
#include "app.h"
#ifdef ENABLE_LOGGING
#define log(...) printf(__VA_ARGS__)
#else
#define log(...)
#endif
//extern float tempFINAL;
//Enum containing temperature read states

typedef enum TempReadSequence
{
	stateIdle,
	stateTimer80ms,
	stateI2CWrite,
	stateTimer10ms,
	stateI2CRead,
}State_t;


#define LETIMER_UF_SCHEDULE (0x1 << 2)
#define LETIMER_COMP1_SCHEDULE (0x1 << 1)
#define I2C_SCHEDULE (0x1)

void schedulerSetEventUF(void);
uint32_t get_event();
bool events_present();
void process_event(uint32_t eventNum);
void schedulerSetEventCOMP1(void);
void schedulerSetEventI2C(void);
void state_machine(struct gecko_cmd_packet* event);


#endif /* SRC_SCHEDULER_H_ */
