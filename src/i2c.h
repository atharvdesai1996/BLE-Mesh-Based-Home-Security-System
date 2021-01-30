/**************************************************************************************************
 * @file      i2c.h
 * @version   0.0.1
 * @brief     i2c.h includes headers and function declarations needed in the file i2c.c
 *
 * @author    Aaksha Jaywant, aaksha.jaywant@colorado.edu
 * @date      Sep 15, 2020
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor  David Sluiter
 *
 * @assignment ecen5823-assignment4-aakshajaywant
 * @due        Sep 25, 2020
 *
 * @resources  Utilized Silicon Labs' EMLIB peripheral libraries to implement functionality.
 *         		1) https://www.silabs.com/community/mcu/32-bit/forum.topic.html/i2c_pin_mapping_loca-ixJK
 * 				2) Lec8 Scheduler Guidance
 * 				3) i2cspm.h used for i2c and Si7021 communication
 * 				4) core_cm4.h used for I2C NVIC enable & disable
 *
 * @copyright  All rights reserved. Distribution allowed only for the use of assignment grading.
 *       Use of code excerpts allowed at the discretion of author. Contact for permission.
 */



#ifndef SRC_I2C_H_
#define SRC_I2C_H_


#include <stdio.h>
#include <stdlib.h>
#include <i2cspm.h>
#include "timers.h"
#define INCLUDE_LOG_DEBUG 1
//#include "log.h"
#include "gpio.h"
#include "node.h"
#ifdef ENABLE_LOGGING
#define log(...) printf(__VA_ARGS__)
#else
#define log(...)
#endif



I2C_TransferReturn_TypeDef retSTAT;
I2C_TransferSeq_TypeDef transferINIT;				//global data structure for access from IRQ handler
I2C_TransferSeq_TypeDef transferINIT1;
uint8_t i2cWrDATA[1];
uint8_t i2cRdDATA[2];

uint16_t tempVAL;
float tempFINAL;

#define tempADDRESS 0x80

void TempReadSequence(void);
void i2cWrite(uint8_t ipData, uint8_t len);
void i2cRead(uint8_t lenRead);
void enableSensor(void);
void disableSensor(void);
void i2cStructInit(void);
I2C_TransferReturn_TypeDef I2CWriteForIRQ(void);
I2C_TransferReturn_TypeDef I2CReadForIRQ(void);
float I2CTempPrint(void);


#endif /* SRC_I2C_H_ */
