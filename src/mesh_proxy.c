/***************************************************************************//**
 * @file  mesh_proxy.c
 * @brief Mesh proxy module implementation
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

 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include "mesh_proxy.h"
#include "lpn.h"

#ifdef ENABLE_LOGGING
#define log(...) printf(__VA_ARGS__)
#else
#define log(...)
#endif

/***************************************************************************//**
 * @addtogroup MeshProxy
 * @{
 ******************************************************************************/

/// number of active mesh proxy connections
uint8_t num_mesh_proxy_conn = 0;

/***************************************************************************//**
 *  Handling of mesh proxy events.
 *
 *  @param[in] pEvt  Pointer to incoming event.
 ******************************************************************************/
void handle_mesh_proxy_events(struct gecko_cmd_packet *pEvt)
{
  switch (BGLIB_MSG_ID(pEvt->header)) {
    case gecko_evt_mesh_proxy_connected_id:
      log("evt:gecko_evt_mesh_proxy_connected_id\r\n");
      num_mesh_proxy_conn++;
      // turn off lpn feature after GATT proxy connection is opened
      lpn_deinit();
      break;

    case gecko_evt_mesh_proxy_disconnected_id:
      log("evt:gecko_evt_mesh_proxy_disconnected_id\r\n");
      if (num_mesh_proxy_conn > 0) {
        if (--num_mesh_proxy_conn == 0) {
          // initialize lpn when there is no active proxy connection
          lpn_init();
        }
      }
      break;
    default:
      break;
  }
}

/** @} (end addtogroup MeshProxy) */
