/***************************************************************************//**
 * @file  lpn.c
 * @brief Low power node implementation file
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
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
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include "display_interface.h"
#include "app_timer.h"
#include "mesh_proxy.h"

#ifdef ENABLE_LOGGING
#define log(...) printf(__VA_ARGS__)
#else
#define log(...)
#endif

/***************************************************************************//**
 * @addtogroup LPN
 * @{
 ******************************************************************************/

/// Flag for indicating that lpn feature is active
static uint8_t lpn_active = 0;

/*******************************************************************************
 * Initialize LPN functionality with configuration and friendship establishment.
 ******************************************************************************/
void lpn_init(void)
{
  uint16_t result = 0;

  // Do not initialize LPN if lpn is currently active
  // or any GATT proxy connection is opened
  if (lpn_active || num_mesh_proxy_conn) {
    return;
  }

  // Initialize LPN functionality.
  result = gecko_cmd_mesh_lpn_init()->result;
  if (result) {
    log("LPN init failed (0x%x)\r\n", result);
    return;
  }
  lpn_active = 1;
  log("LPN initialized\r\n");
  DI_Print("LPN on", DI_ROW_LPN);

  // Configure LPN Minimum friend queue length = 2
  result = gecko_cmd_mesh_lpn_config(mesh_lpn_queue_length, 2)->result;
  if (result) {
    log("LPN queue configuration failed (0x%x)\r\n", result);
    return;
  }
  // Configure LPN Poll timeout = 5 seconds
  result = gecko_cmd_mesh_lpn_config(mesh_lpn_poll_timeout, 5 * 1000)->result;
  if (result) {
    log("LPN Poll timeout configuration failed (0x%x)\r\n", result);
    return;
  }
  log("trying to find friend...\r\n");
  result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

  if (result != 0) {
    log("ret.code 0x%x\r\n", result);
  }
}

/*******************************************************************************
 * Deinitialize LPN functionality.
 ******************************************************************************/
void lpn_deinit(void)
{
  uint16_t result = 0;

  if (!lpn_active) {
    return; // lpn feature is currently inactive
  }

  // Cancel friend finding timer
  result = gecko_cmd_hardware_set_soft_timer(TIMER_STOP,
                                             FRIEND_FIND_TIMER,
                                             1)->result;

  // Terminate friendship if exist
  result = gecko_cmd_mesh_lpn_terminate_friendship()->result;
  if (result) {
    log("Friendship termination failed (0x%x)\r\n", result);
  }
  // turn off lpn feature
  result = gecko_cmd_mesh_lpn_deinit()->result;
  if (result) {
    log("LPN deinit failed (0x%x)\r\n", result);
  }
  lpn_active = 0;
  log("LPN deinitialized\r\n");
  DI_Print("LPN off", DI_ROW_LPN);
}

/*******************************************************************************
 *  Handling of mesh lpn events.
 *  It handles:
 *   - lpn_friendship_established_id
 *   - lpn_friendship_failed_id
 *   - lpn_friendship_terminated_id
 *
 *  @param[in] pEvt  Pointer to incoming lpn event.
 ******************************************************************************/
void handle_lpn_events(struct gecko_cmd_packet *pEvt)
{
  uint16_t result = 0;

  switch (BGLIB_MSG_ID(pEvt->header)) {
    case gecko_evt_mesh_lpn_friendship_established_id:
      log("friendship established\r\n");
      DI_Print("LPN with friend", DI_ROW_LPN);
      break;

    case gecko_evt_mesh_lpn_friendship_failed_id:
      log("friendship failed\r\n");
      DI_Print("no friend", DI_ROW_LPN);
      // try again in 2 seconds
      result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
                                                 FRIEND_FIND_TIMER,
                                                 SINGLE_SHOT)->result;
      if (result) {
        log("timer failure?!  0x%x\r\n", result);
      }
      break;

    case gecko_evt_mesh_lpn_friendship_terminated_id:
      log("friendship terminated\r\n");
      DI_Print("friend lost", DI_ROW_LPN);
      if (num_mesh_proxy_conn == 0) {
        // try again in 2 seconds
        result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
                                                   FRIEND_FIND_TIMER,
                                                   SINGLE_SHOT)->result;
        if (result) {
          log("timer failure?!  0x%x\r\n", result);
        }
      }
      break;

    default:
      break;
  }
}

/*******************************************************************************
 *  Handling of lpn timer events.
 *
 *  @param[in] pEvt  Pointer to incoming event.
 ******************************************************************************/
void handle_lpn_timer_evt(struct gecko_cmd_packet *pEvt)
{
  uint16_t result = 0;
  switch (pEvt->data.evt_hardware_soft_timer.handle) {
    case NODE_CONFIGURED_TIMER:
      if (!lpn_active) {
        log("try to initialize lpn...\r\n");
        lpn_init();
      }
      break;

    case FRIEND_FIND_TIMER:
    {
      log("trying to find friend...\r\n");
      result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

      if (result != 0) {
        log("ret.code 0x%x\r\n", result);
      }
    }
    break;

    default:
      break;
  }
}

/*******************************************************************************
 *  Set the timer that delay LPN initialization to enable quick configuration
 *  over advertising bearer.
 *
 *  @param[in] delay  Time to set for the timer.
 ******************************************************************************/
void set_configuration_timer(uint32_t delay)
{
  uint16_t result;
  result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delay),
                                             NODE_CONFIGURED_TIMER,
                                             SINGLE_SHOT)->result;
  if (result) {
    log("timer failure?!  0x%x\r\n", result);
  }
}

/** @} (end addtogroup LPN) */
