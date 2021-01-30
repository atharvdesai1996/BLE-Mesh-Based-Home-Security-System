/***************************************************************************//**
 * @file  gpio.h was buttons.h from Silabs soc-btmest-switch example code
 * @brief gpio header file
 *
 * @editor    Atharv Desai, atharv.desai@colorado.edu
 * @date      Nov 30, 2020
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor  David Sluiter
 *
 * @assignment Final Project
 * @due        Dec 4, 2020
 *
 * @resources  Utilized Silicon Labs' BT mesh v1.7 library
 *
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef BUTTONS_H
#define BUTTONS_H

/* Retarget serial headers */
#include "retargetserial.h"
#include <stdio.h>
#include "em_cmu.h"
#include "ble_mesh_device_type.h"
#ifdef ENABLE_LOGGING
#define log(...) printf(__VA_ARGS__)
#else
#define log(...)
#endif


/*******************************************************************************
 * External signal definitions. These are used to signal button press events
 * from GPIO interrupt handler to application.
 ******************************************************************************/



#define EXT_SIGNAL_PB0_PRESS             0x01
#define EXT_SIGNAL_PB0_RELEASE           0x02
#define EXT_SIGNAL_PB1_PRESS             0x04
#define EXT_SIGNAL_PB1_RELEASE           0x08
#define EXT_SIGNAL_PIR                   16
#define EXT_SIGNAL_NOISE 				 32

/***************************************************************************//**
 * Button initialization. Configure pushbuttons PB0, PB1 as inputs.
 ******************************************************************************/
void button_init(void);

/***************************************************************************//**
 * Enable button interrupts for PB0, PB1. Both GPIOs are configured to trigger
 * an interrupt on the rising and falling edges + enable input glitch filtering.
 ******************************************************************************/
void enable_button_interrupts(void);

#if NOISE_SENSOR == 1
	#include <stdint.h>
	#define soundPort gpioPortA
	#define soundGate 3
	uint32_t GPIOsound;
	void sound_init(void);
	void enable_sound_interrupts(void);
	void sound_interrupt(void);

#elif PIR_SENSOR == 1
//	#define	PIRLED_port gpioPortC   // Pin P5 on the breakout board
	//#define PIRLED_pin  8
#define	PIRLED_port gpioPortA   // Pin P5 on the breakout board
#define PIRLED_pin  3
	#define	PIRLED_port1 gpioPortC   // Pin P7 on the breakout board
	#define PIRLED_pin1  9
	void gpioLedPIRSetOn(void);
	void gpioLedPIRSetOff(void);
	void gpioInit(void);
	void pir_init(void);
	void pir_interrupt(void);
#endif

#endif /* BUTTONS_H */
