/***************************************************************************//**
 * @file  app.h from Silabs soc-btmest-switch example code
 * @brief app header file
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

#ifndef APP_H
#define APP_H

#include <gecko_configuration.h>

// DOS, added to enable #define to control whether a publisher or a subscriber
// is implemented.
#include <src/ble_mesh_device_type.h>

/* Bluetooth stack headers */
#include <src/gpio.h>
#include <src/node.h>
#include "native_gecko.h"
#include "gatt_db.h"
#include "mesh_proxy.h"
#include "lpn.h"

/* Timer definitions */
#include "app_timer.h"

#include "app.h"

//////////////////////////////////////////////////
#include "sleep.h"
#include "oscillators.h"
#include "timers.h"
#include "em_letimer.h"
//////////////////////////////////////////////////////


/* Coex header */
#include "coexistence-ble.h"

/* Display Interface header */
#include "display_interface.h"

/* Retarget serial headers */
#include "retargetserial.h"
#include <stdio.h>


void SleepInitFUNC(void);
void DeepSleep(void);
/* Uncomment one  of the energy modes as per requirement */
//#define LOWEST_ENERGY_MODE 0
//#define LOWEST_ENERGY_MODE 1
#define LOWEST_ENERGY_MODE 2
//#define LOWEST_ENERGY_MODE 3

#define ENABLE_SLEEPING 0
////////////////////////////////////////////////////

/***************************************************************************//**
 * Main application code.
 * @param[in] pConfig  Pointer to stack configuration.
 ******************************************************************************/
void appMain(const gecko_configuration_t *pConfig);

int get_alarm_deactivate();



//uint32_t frequency;



#endif /* APP_H */
