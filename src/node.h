/***************************************************************************
 * @file  node.h was switch.h from Silabs soc-btmest-switch example code
 * @brief node module header file
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
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
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

#ifndef SWITCH_H
#define SWITCH_H

#include <stdint.h>
#include "native_gecko.h"
#include "display_interface.h"



/*******************************************************************************
   Public Macros and Definitions
*******************************************************************************/

#define OFF         0   ///< Set switch state to off
#define ON          1   ///< Set switch state to on


/***************************************************************************//**
 * Switch node initialization.
 * This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 ******************************************************************************/
void node_init(void);



/***************************************************************************//**
 * This function change the switch position and send it to the server.
 *
 * @param[in] position  Defines switch position change, possible values are
 *                      0 = OFF (released), 1 = ON (pressed)
 *
 ******************************************************************************/
void change_switch_position(uint8_t position);



/***************************************************************************//**
 *  Handling of message retransmission timer events.
 *
 *  @param[in] pEvt  Pointer to incoming event.
 ******************************************************************************/
void handle_retrans_timer_evt(struct gecko_cmd_packet *pEvt);

static uint8_t storage_val[1];

void send_level_request(uint16_t temp_data, uint16_t delay);

#endif /* SWITCH_H */
