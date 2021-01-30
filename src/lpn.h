/***************************************************************************//**
 * @file  lpn.h
 * @brief Low power node header file
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
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
 *
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef LPN_H
#define LPN_H

/***************************************************************************//**
 * @defgroup LPN Low Power Node Module
 * @brief Low Power Node Module Implementation
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup LPN
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * Initialize LPN functionality with configuration and friendship establishment.
 ******************************************************************************/
void lpn_init(void);

/***************************************************************************//**
 * Deinitialize LPN functionality.
 ******************************************************************************/
void lpn_deinit(void);

/***************************************************************************//**
 *  Handling of mesh lpn events.
 *  It handles:
 *   - lpn_friendship_established_id
 *   - lpn_friendship_failed_id
 *   - lpn_friendship_terminated_id
 *
 *  @param[in] pEvt  Pointer to incoming lpn event.
 ******************************************************************************/
void handle_lpn_events(struct gecko_cmd_packet *pEvt);

/***************************************************************************//**
 *  Handling of lpn timer events.
 *
 *  @param[in] pEvt  Pointer to incoming event.
 ******************************************************************************/
void handle_lpn_timer_evt(struct gecko_cmd_packet *pEvt);

/***************************************************************************//**
 *  Set the timer that delay LPN initialization to enable quick configuration
 *  over advertising bearer.
 *
 *  @param[in] delay  Time to set for the timer.
 ******************************************************************************/
void set_configuration_timer(uint32_t delay);

/** @} (end addtogroup LPN) */

#endif /* LPN_H */
