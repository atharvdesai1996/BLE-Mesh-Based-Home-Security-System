/***************************************************************************//**
 * @file  app.c from Silabs soc-btmest-switch example code
 * @brief app implementation file
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


#include "app.h"
#include "alarm.h"
#include "scheduler.h"

#ifdef ENABLE_LOGGING
#define log(...) printf(__VA_ARGS__)
#else
#define log(...)
#endif

#define DEACTIVATE_TIME_S 10 //#seconds to deactivate alarms, in units of seconds

/*******************************************************************************
 * Provisioning bearers defines.
 ******************************************************************************/
#define PB_ADV   0x1 ///< Advertising Provisioning Bearer
#define PB_GATT  0x2 ///< GATT Provisioning Bearer

/// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

/*******************************************************************************
 * Global variables
 ******************************************************************************/
/// Address of the Primary Element of the Node
static uint16_t _my_address = 0;
/// number of active Bluetooth connections
static uint8_t num_connections = 0;
/// handle of the last opened LE connection
static uint8_t conn_handle = 0xFF;
/// Flag for indicating that provisioning procedure is finished
static uint8_t provisioning_finished = 0;



#if TEMP_SENSOR || ENABLE_SLEEPING
uint32_t selectedFREQ;
#endif

extern uint8_t storage_val[1];
/*******************************************************************************
 * Function prototypes.
 ******************************************************************************/
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *pEvt);

/***************************************************************************//**
 * Initialise used bgapi classes.
 ******************************************************************************/
static void gecko_bgapi_classes_init(void)
{


#if FRIEND_NODE
	/**********************************************************************************************/
						// Friend Node Init Code //

	/**********************************************************************************************/
	  gecko_bgapi_class_dfu_init();
	  gecko_bgapi_class_system_init();
	  gecko_bgapi_class_le_gap_init();
	  gecko_bgapi_class_le_connection_init();
	  //gecko_bgapi_class_gatt_init();
	  gecko_bgapi_class_gatt_server_init();
	  gecko_bgapi_class_hardware_init();
	  gecko_bgapi_class_flash_init();
	  gecko_bgapi_class_test_init();
	  //gecko_bgapi_class_sm_init();
	  gecko_bgapi_class_mesh_node_init();
	  //gecko_bgapi_class_mesh_prov_init();
	  gecko_bgapi_class_mesh_proxy_init();
	  gecko_bgapi_class_mesh_proxy_server_init();
	  //gecko_bgapi_class_mesh_proxy_client_init();
	  //gecko_bgapi_class_mesh_generic_client_init();
	  gecko_bgapi_class_mesh_generic_server_init();
	  //gecko_bgapi_class_mesh_vendor_model_init();
	  //gecko_bgapi_class_mesh_health_client_init();
	  //gecko_bgapi_class_mesh_health_server_i it();
	  //gecko_bgapi_class_mesh_test_init();
	  //gecko_bgapi_class_mesh_lpn_init();
	  gecko_bgapi_class_mesh_friend_init();
	  gecko_bgapi_class_mesh_lc_server_init();
	  gecko_bgapi_class_mesh_lc_setup_server_init();
	  gecko_bgapi_class_mesh_scene_server_init();
	  gecko_bgapi_class_mesh_scene_setup_server_init();
	  gecko_bgapi_class_mesh_time_server_init();
	  gecko_bgapi_class_mesh_scheduler_server_init();
#else



	  gecko_bgapi_class_dfu_init();
	  gecko_bgapi_class_system_init();
	  gecko_bgapi_class_le_gap_init();
	  gecko_bgapi_class_le_connection_init();
	  gecko_bgapi_class_gatt_server_init();
	  gecko_bgapi_class_hardware_init();
	  gecko_bgapi_class_flash_init();
	  gecko_bgapi_class_test_init();
	  gecko_bgapi_class_mesh_node_init();
	  gecko_bgapi_class_mesh_proxy_init();
	  gecko_bgapi_class_mesh_proxy_server_init();
	  gecko_bgapi_class_mesh_lpn_init();
	  gecko_bgapi_class_mesh_generic_client_init();
	  //gecko_bgapi_class_mesh_scene_client_init();

#endif
}


///////////////////////////////////////

void SleepInitFUNC(void)
{

	SLEEP_Init_t initSLEEP_obj;
	memset(&initSLEEP_obj, 0, sizeof(initSLEEP_obj));				//Initialize SLEEP_Init_t to 0
	SLEEP_InitEx(&initSLEEP_obj);
	SLEEP_SleepBlockBegin(LOWEST_ENERGY_MODE + 1);


}


/////////////////////////////////////////





/*******************************************************************************
 * Main application code.
 * @param[in] pConfig  Pointer to stack configuration.
 ******************************************************************************/
void appMain(const gecko_configuration_t *pConfig)
{
  // Initialize stack
  gecko_stack_init(pConfig);
  gecko_bgapi_classes_init();

  // Initialize the random number generator which is needed for proper radio work.
  gecko_cmd_system_get_random_data(16);

  // Initialize coexistence interface. Parameters are taken from HAL config.
  gecko_initCoexHAL();

  // Initialize debug prints and display interface
  RETARGET_SerialInit();


#if !PIR_SENSOR
  DI_Init();
#endif


#if TEMP_SENSOR
  selectedFREQ = selectLXFO();
 	         	 initialize_LETIMER0(selectedFREQ);
 	         	i2cStructInit();
#endif

#if NOISE_SENSOR
  sound_init();
#endif

#if PIR_SENSOR == 1
  gpioInit();
  //pir_init();

#endif

  // Initialize buttons. Note: some radio boards share the same GPIO
  // for button & LED. Initialization is done in this order so that default
  // configuration will be "button" for those radio boards with shared pins.
  button_init();

#if PIR_SENSOR == 1
  DI_Init();
#endif


  /////////////////////////////
  //Osc_clock_select();  // Enabling the oscillator
  //mysleep();           // Initialize the sleep



  ///////////////////////////////////////////
#if (ENABLE_SLEEPING)

#if LOWEST_ENERGY_MODE == 0
				selectedFREQ = selectLXFO();							//selects the LXFO oscillator
			#endif

			#if LOWEST_ENERGY_MODE == 1
				selectedFREQ = selectLXFO();							//selects the LXFO oscillator
			#endif

			#if LOWEST_ENERGY_MODE == 2
				selectedFREQ = selectLXFO();							//selects the LXFO oscillator
			#endif

			#if LOWEST_ENERGY_MODE == 3
				selectedFREQ = selectULFRCO();							//selects the ULFRCO oscillator
			#endif


initialize_LETIMER0(selectedFREQ);

SleepInitFUNC();
#endif
  ///////////////////////////////////////////


  while (1) {
    // Event pointer for handling events
    struct gecko_cmd_packet* evt;

    // If there are no events pending then the next call to gecko_wait_event()
    // may cause device to go to deep sleep.
    // Make sure that debug prints are flushed before going to sleep
    if (!gecko_event_pending()) {
      RETARGET_SerialFlush();
    }

    // Check for stack event
    evt = gecko_wait_event();

    bool pass = mesh_bgapi_listener(evt);
    if (pass) {
      handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
    }
		#if TEMP_SENSOR
    		state_machine(evt);
		#endif
  }
}

/***************************************************************************//**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 *
 * @param[in] pAddr  Pointer to Bluetooth address.
 ******************************************************************************/
static void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16_t res;


#if FRIEND_NODE
  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "friend node %02x:%02x", pAddr->addr[1], pAddr->addr[0]);

  log("Device name: '%s'\r\n", name);

  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8_t *)name)->result;
  if (res) {
    log("gecko_cmd_gatt_server_write_attribute_value() failed, code 0x%x\r\n", res);
  }

  // show device name on the LCD
  DI_Print(name, DI_ROW_NAME);

#else
  // create unique device name using the last two bytes of the Bluetooth address
#if PIR_SENSOR == 1
  sprintf(name, "PIR node %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
#endif

#if NOISE_SENSOR
  sprintf(name, "SOUND node %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
#endif

  log("Device name: '%s'\r\n", name);

  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8_t *)name)->result;
  if (res) {
    log("gecko_cmd_gatt_server_write_attribute_value() failed, code 0x%x\r\n", res);
  }

  // show device name on the LCD
  DI_Print(name, DI_ROW_NAME);
#endif
}

/***************************************************************************//**
 * This function is called to initiate factory reset. Factory reset may be
 * initiated by keeping one of the WSTK pushbuttons pressed during reboot.
 * Factory reset is also performed if it is requested by the provisioner
 * (event gecko_evt_mesh_node_reset_id).
 ******************************************************************************/
static void initiate_factory_reset(void)
{
  log("factory reset\r\n");
  DI_Print("\n***\nFACTORY RESET\n***", DI_ROW_STATUS);

  /* if connection is open then close it before rebooting */
  if (conn_handle != 0xFF) {
    gecko_cmd_le_connection_close(conn_handle);
  }

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
                                    FACTORY_RESET_TIMER,
                                    SINGLE_SHOT);
}

/***************************************************************************//**
 * Handling of stack events. Both Bluetooth LE and Bluetooth mesh events
 * are handled here.
 * @param[in] evt_id  Incoming event ID.
 * @param[in] pEvt    Pointer to incoming event.
 ******************************************************************************/
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *pEvt)
{
#if FRIEND_NODE
	/**********************************************************************************************/
						// Friend Node Event Code //

						// Author: Joe Lopez	  //
	/**********************************************************************************************/
	  uint16_t result;
	  char buf[30]; // for sprintf, see below

	  if (NULL == pEvt) {
	    return;
	  }

	  switch (evt_id) {

	    case gecko_evt_system_boot_id:

		  set_alarm_state(0);
		  set_alarm_deactivate(0);

	      // check pushbutton state at startup. If PB0 is held down then do factory reset
	      if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0) {

	    	  initiate_factory_reset();


	    	  storage_val[0] = 0;
			  struct gecko_msg_flash_ps_save_rsp_t * store_result = gecko_cmd_flash_ps_save(STORAGE_KEY, 1, storage_val);
			  if(store_result->result){
				  log("Persistent storage store failed. Error code: %x\n\r", store_result->result);
			  DI_Print("Total Alarms: 0", 8);

		  }


	      } else {

	        struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

	        set_device_name(&pAddr->address);

	        // Initialize Mesh stack in Node operation mode, it will generate initialized event
	        // gecko_evt_mesh_node_initialized_id
	        result = gecko_cmd_mesh_node_init()->result;
	        if (result) {
	            log ("init failed (0x%x)", result); // DOS
	            sprintf(buf, "init failed (0x%x)", result);
	            DI_Print(buf, DI_ROW_STATUS);
	        }

	      }

	      break;

	    case gecko_evt_hardware_soft_timer_id:

	      switch (pEvt->data.evt_hardware_soft_timer.handle) {

	        // --------------------------------------------
	        // see app_timer.h for #defines of these timers
	        // --------------------------------------------
	        case FACTORY_RESET_TIMER:
	          // reset the device to finish factory reset
	          gecko_cmd_system_reset(0);
	          break;

	        case RESTART_TIMER:
	          // restart timer expires, reset the device
	          gecko_cmd_system_reset(0);
	          break;

	        case RETRANS_ONOFF_TIMER:
	          handle_retrans_timer_evt(pEvt);
	          break;

	        case NODE_CONFIGURED_TIMER:
	          break;

	        case ALARM_DEACTIVATE_TIMER:
	        	set_ad_counter(get_ad_counter()-1);

	        	if(get_ad_counter()){   //check and see if we're out of deactivate time
	        		sprintf(buf, "%d", get_ad_counter());
	        		DI_Print(buf, 6);
				    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(1000),  //start another 1-sec timer
	    		                                        ALARM_DEACTIVATE_TIMER,
	    		                                        SINGLE_SHOT);
	        	}
	        	else{
					set_alarm_deactivate(0);		//time's up! reactivate alarm
					log("Alarm re-activated\n\r");
					DI_Print("Alarm Not Active", DI_ROW_LIGHTNESS);  //go back to 'not active' state
					DI_Print("" , 6);  //clear timer row (row 6)
	        	}
	        	break;

	        default:
	          break;
	      }

	      break;


	    case gecko_evt_mesh_node_initialized_id:
	      log("node initialized\r\n");

	      errorcode_t     result;

	      if(DeviceUsesClientModel()){
	          result = gecko_cmd_mesh_generic_client_init()->result;
	          if (result) {
	              log("gecko_cmd_mesh_generic_client_init failed, code 0x%x\r\n", result);
	          }
	      }
	      if(DeviceUsesServerModel()){
	          result = gecko_cmd_mesh_generic_server_init()->result;
	          if (result) {
	              log("gecko_cmd_mesh_generic_server_init failed, code 0x%x\r\n", result);
	          }
	          else{
	        	  log("mesh server init succeeded\n\r");
	          }


	      }
	      // Initialize generic client models


	      struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(pEvt->data);

	      if (pData->provisioned) {
	        log("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);

	        _my_address = pData->address;

	        enable_button_interrupts();

	        provisioning_finished = 1;
	        node_init();

	        DI_Print("provisioned", DI_ROW_STATUS);
	        DI_Print("Alarm Not Active", DI_ROW_LIGHTNESS);
	      } else {

	        log("node is unprovisioned\r\n");
	        DI_Print("unprovisioned", DI_ROW_STATUS);

	        log("starting unprovisioned beaconing...\r\n");
	        // Enable ADV and GATT provisioning bearer
	        gecko_cmd_mesh_node_start_unprov_beaconing(PB_ADV | PB_GATT);
	      }


	      break;


	    case gecko_evt_system_external_signal_id:
	    {

	    	int event = pEvt->data.evt_system_external_signal.extsignals;
	    	if(event == EXT_SIGNAL_PB0_PRESS){
	    		sprintf(buf, "PB0 Pressed\n\r");
	    		log(buf);
	    		if(get_alarm_state()){
	    			set_alarm_state(0) ;
	    			DI_Print("Alarm Not Active", DI_ROW_LIGHTNESS);
	    		}
	    		else if(!get_alarm_deactivate()){
	    			set_alarm_deactivate(1);
	    			DI_Print("Alarm Deactivated", DI_ROW_LIGHTNESS);
	    			set_ad_counter(DEACTIVATE_TIME_S);
				    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(1000), //start the first timer counter, 1 second
	    		                                        ALARM_DEACTIVATE_TIMER,
	    		                                        SINGLE_SHOT);
				    sprintf(buf, "%d", DEACTIVATE_TIME_S); //display how much deactivate time is left (at this point the full 10 sec)
				    DI_Print(buf,  6);
	    		}

	    	}
	    	if(event == EXT_SIGNAL_PB0_RELEASE){
	    		sprintf(buf, "PB0 Released\n\r");
	    		log(buf);
	    	}
	    	if(event == EXT_SIGNAL_PB1_PRESS){
	    		sprintf(buf, "PB1 Pressed\n\r");
	    		log(buf);
	    	}


	    }

	    break;


	    case gecko_evt_mesh_node_provisioning_started_id:

	      log("Started provisioning\r\n");
	      DI_Print("provisioning...", DI_ROW_STATUS);

	      break;


	    case gecko_evt_mesh_node_provisioned_id:

	      provisioning_finished = 1;
	      node_init();

	      log("node provisioned, got address=%x\r\n", pEvt->data.evt_mesh_node_provisioned.address);

	      DI_Print("provisioned", DI_ROW_STATUS);

	      enable_button_interrupts();

	      break;


	    case gecko_evt_mesh_node_provisioning_failed_id:

	      log("provisioning failed, code 0x%x\r\n", pEvt->data.evt_mesh_node_provisioning_failed.result);
	      DI_Print("prov failed", DI_ROW_STATUS);

	      /* start a one-shot timer that will trigger soft reset after small delay */
	      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
	                                        RESTART_TIMER,
	                                        SINGLE_SHOT);
	      break;


	    case gecko_evt_mesh_node_key_added_id:

	      log("got new %s key with index 0x%x\r\n",
	          pEvt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
	          pEvt->data.evt_mesh_node_key_added.index);

	      break;


	    case gecko_evt_mesh_node_model_config_changed_id:

	      log("model config changed\r\n");

	      break;


	    case gecko_evt_mesh_node_config_set_id:

	      log("model config set\r\n");

	      break;


	    case gecko_evt_le_connection_opened_id:

	      log("evt:gecko_evt_le_connection_opened_id\r\n");
	      num_connections++;
	      conn_handle = pEvt->data.evt_le_connection_opened.connection;
	      DI_Print("connected", DI_ROW_CONNECTION);

	      break;


	    case gecko_evt_le_connection_closed_id:

	      /* Check if need to boot to dfu mode */
	      if (boot_to_dfu) {
	        /* Enter to DFU OTA mode */
	        gecko_cmd_system_reset(2);
	      }

	      log("evt:conn closed, reason 0x%x\r\n", pEvt->data.evt_le_connection_closed.reason);
	      conn_handle = 0xFF;
	      if (num_connections > 0) {
	        if (--num_connections == 0) {
	          DI_Print("", DI_ROW_CONNECTION);
	        }
	      }

	      break;


	    case gecko_evt_mesh_node_reset_id:

	      log("evt gecko_evt_mesh_node_reset_id\r\n");
	      initiate_factory_reset();

	      break;


	    case gecko_evt_le_connection_parameters_id:

	      log("connection params: interval %d, timeout %d\r\n",
	          pEvt->data.evt_le_connection_parameters.interval,
	          pEvt->data.evt_le_connection_parameters.timeout);

	      break;


	    case gecko_evt_le_gap_adv_timeout_id:
	      // these events silently discarded
	      break;


	    case gecko_evt_gatt_server_user_write_request_id:

	      if (pEvt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
	        /* Set flag to enter to OTA mode */
	        boot_to_dfu = 1;
	        /* Send response to Write Request */
	        gecko_cmd_gatt_server_send_user_write_response(
	          pEvt->data.evt_gatt_server_user_write_request.connection,
	          gattdb_ota_control,
	          bg_err_success);

	        /* Close connection to enter to DFU OTA mode */
	        gecko_cmd_le_connection_close(pEvt->data.evt_gatt_server_user_write_request.connection);
	      }

	      break;


	    case gecko_evt_mesh_generic_server_client_request_id:
	    	log("Server Client Request\n\r");
	    	mesh_lib_generic_server_event_handler(pEvt);
	    	break;
	    case gecko_evt_mesh_generic_server_state_changed_id:
	    	log("Server State Changed\n\r");
	    	mesh_lib_generic_server_event_handler(pEvt);
	    	break;
	    case gecko_evt_mesh_generic_server_state_recall_id:
	    	log("Server State Recall\n\r");
	    	mesh_lib_generic_server_event_handler(pEvt);
	    	break;

	    case gecko_evt_mesh_friend_friendship_established_id:
	      log("evt gecko_evt_mesh_friend_friendship_established, lpn_address=%x\r\n", pEvt->data.evt_mesh_friend_friendship_established.lpn_address);
	      DI_Print("FRIEND", DI_ROW_FRIEND);
	      break;

	    case gecko_evt_mesh_friend_friendship_terminated_id:
	      log("evt gecko_evt_mesh_friend_friendship_terminated, reason=%x\r\n", pEvt->data.evt_mesh_friend_friendship_terminated.reason);
	      DI_Print("NO LPN", DI_ROW_FRIEND);
	      break;


	    default:
	      //log("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
	      break;
	  }
#endif //FRIEND_NODE

#if LIGHT_SENSOR
	  uint16_t result;
	  char buf[30]; // for sprintf, see below

	  if (NULL == pEvt) {
	    return;
	  }

	  switch (evt_id) {

	    case gecko_evt_system_boot_id:

	      // check pushbutton state at startup. If PB0 is held down then do factory reset
	      if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0) {

	        initiate_factory_reset();

	      } else {

	        struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

	        set_device_name(&pAddr->address);

	        // Initialize Mesh stack in Node operation mode, it will generate initialized event
	        // gecko_evt_mesh_node_initialized_id
	        result = gecko_cmd_mesh_node_init()->result;
	        if (result) {
	            log ("init failed (0x%x)", result); // DOS
	            sprintf(buf, "init failed (0x%x)", result);
	            DI_Print(buf, DI_ROW_STATUS);
	        }

	      }

	      break;

	    case gecko_evt_hardware_soft_timer_id:

	      switch (pEvt->data.evt_hardware_soft_timer.handle) {

	        // --------------------------------------------
	        // see app_timer.h for #defines of these timers
	        // --------------------------------------------
	        case FACTORY_RESET_TIMER:
	          // reset the device to finish factory reset
	          gecko_cmd_system_reset(0);
	          break;

	        case RESTART_TIMER:
	          // restart timer expires, reset the device
	          gecko_cmd_system_reset(0);
	          break;

	        case RETRANS_ONOFF_TIMER:
	          handle_retrans_timer_evt(pEvt);
	          break;

	        case NODE_CONFIGURED_TIMER:
	          break;

	        default:
	          break;
	      }

	      break;


	    case gecko_evt_mesh_node_initialized_id:
	      log("node initialized\r\n");

	      errorcode_t     result;

	      // Initialize generic client models
	      result = gecko_cmd_mesh_generic_client_init()->result;
	      if (result) {
	          log("gecko_cmd_mesh_generic_client_init failed, code 0x%x\r\n", result);
	      }

	      struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(pEvt->data);

	      if (pData->provisioned) {
	        log("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);

	        _my_address = pData->address;

	        enable_button_interrupts();

	        provisioning_finished = 1;
	        node_init();

	        DI_Print("provisioned", DI_ROW_STATUS);

	      } else {

	        log("node is unprovisioned\r\n");
	        DI_Print("unprovisioned", DI_ROW_STATUS);

	        log("starting unprovisioned beaconing...\r\n");
	        // Enable ADV and GATT provisioning bearer
	        gecko_cmd_mesh_node_start_unprov_beaconing(PB_ADV | PB_GATT);
	      }

	      break;


	    case gecko_evt_system_external_signal_id:
	    {

	    	// edit #2:
	    	// for each PB0 event (press and release) call change_switch_position()
	    	// with the appropriate parameter. change_switch_position() is defined in
	    	// node.c






	    }

	    break;


	    case gecko_evt_mesh_node_provisioning_started_id:

	      log("Started provisioning\r\n");
	      DI_Print("provisioning...", DI_ROW_STATUS);

	      break;


	    case gecko_evt_mesh_node_provisioned_id:

	      provisioning_finished = 1;
	      node_init();

	      log("node provisioned, got address=%x\r\n", pEvt->data.evt_mesh_node_provisioned.address);

	      DI_Print("provisioned", DI_ROW_STATUS);

	      enable_button_interrupts();

	      break;


	    case gecko_evt_mesh_node_provisioning_failed_id:

	      log("provisioning failed, code 0x%x\r\n", pEvt->data.evt_mesh_node_provisioning_failed.result);
	      DI_Print("prov failed", DI_ROW_STATUS);

	      /* start a one-shot timer that will trigger soft reset after small delay */
	      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
	                                        RESTART_TIMER,
	                                        SINGLE_SHOT);
	      break;


	    case gecko_evt_mesh_node_key_added_id:

	      log("got new %s key with index 0x%x\r\n",
	          pEvt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
	          pEvt->data.evt_mesh_node_key_added.index);

	      break;


	    case gecko_evt_mesh_node_model_config_changed_id:

	      log("model config changed\r\n");

	      break;


	    case gecko_evt_mesh_node_config_set_id:

	      log("model config set\r\n");

	      break;


	    case gecko_evt_le_connection_opened_id:

	      log("evt:gecko_evt_le_connection_opened_id\r\n");
	      num_connections++;
	      conn_handle = pEvt->data.evt_le_connection_opened.connection;
	      DI_Print("connected", DI_ROW_CONNECTION);

	      break;


	    case gecko_evt_le_connection_closed_id:

	      /* Check if need to boot to dfu mode */
	      if (boot_to_dfu) {
	        /* Enter to DFU OTA mode */
	        gecko_cmd_system_reset(2);
	      }

	      log("evt:conn closed, reason 0x%x\r\n", pEvt->data.evt_le_connection_closed.reason);
	      conn_handle = 0xFF;
	      if (num_connections > 0) {
	        if (--num_connections == 0) {
	          DI_Print("", DI_ROW_CONNECTION);
	        }
	      }

	      break;


	    case gecko_evt_mesh_node_reset_id:

	      log("evt gecko_evt_mesh_node_reset_id\r\n");
	      initiate_factory_reset();

	      break;


	    case gecko_evt_le_connection_parameters_id:

	      log("connection params: interval %d, timeout %d\r\n",
	          pEvt->data.evt_le_connection_parameters.interval,
	          pEvt->data.evt_le_connection_parameters.timeout);

	      break;


	    case gecko_evt_le_gap_adv_timeout_id:
	      // these events silently discarded
	      break;


	    case gecko_evt_gatt_server_user_write_request_id:

	      if (pEvt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
	        /* Set flag to enter to OTA mode */
	        boot_to_dfu = 1;
	        /* Send response to Write Request */
	        gecko_cmd_gatt_server_send_user_write_response(
	          pEvt->data.evt_gatt_server_user_write_request.connection,
	          gattdb_ota_control,
	          bg_err_success);

	        /* Close connection to enter to DFU OTA mode */
	        gecko_cmd_le_connection_close(pEvt->data.evt_gatt_server_user_write_request.connection);
	      }

	      break;





	    default:
	      //log("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
	      break;
	  }
#endif //LIGHT_SENSOR

#if NOISE_SENSOR
	  uint16_t result;
	   char buf[30];

	   if (NULL == pEvt) {
	     return;
	   }

	   switch (evt_id) {
	     case gecko_evt_system_boot_id:
	       // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
	       if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0
	           ) {
	         initiate_factory_reset();
	       } else {
	         struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

	         set_device_name(&pAddr->address);

	         // Initialize Mesh stack in Node operation mode, it will generate initialized event
	         result = gecko_cmd_mesh_node_init()->result;
	         if (result) {
	           sprintf(buf, "init failed (0x%x)", result);
	           DI_Print(buf, DI_ROW_STATUS);
	         }
	       }
	       break;

	     case gecko_evt_hardware_soft_timer_id:
	       switch (pEvt->data.evt_hardware_soft_timer.handle) {
	         case FACTORY_RESET_TIMER:
	           // reset the device to finish factory reset
	           gecko_cmd_system_reset(0);
	           break;

	         case RESTART_TIMER:
	           // restart timer expires, reset the device
	           gecko_cmd_system_reset(0);
	           break;

	         case PROVISIONING_TIMER:
	           // toggle LED to indicate the provisioning state
	           if (!provisioning_finished) {
	             //led_set_state(LED_STATE_PROV);
	           }
	           break;

	         case RETRANS_ONOFF_TIMER:
	           handle_retrans_timer_evt(pEvt);
	           break;

	         case NODE_CONFIGURED_TIMER:
	         case FRIEND_FIND_TIMER:
	           handle_lpn_timer_evt(pEvt);
	           break;

	         default:
	           break;
	       }

	       break;

	     case gecko_evt_mesh_node_initialized_id:
	       log("node initialized\r\n");

	       // Initialize generic client models
	       result = gecko_cmd_mesh_generic_client_init_on_off()->result;
	       if (result) {
	         log("mesh_generic_client_init_on_off failed, code 0x%x\r\n", result);
	       }

	       result = gecko_cmd_mesh_generic_client_init_level()->result;
	       if (result) {
	      	         log("mesh_generic_client_init_level failed, code 0x%x\r\n", result);
	      	       }

	       gecko_cmd_mesh_generic_client_init();


	       struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(pEvt->data);

	       if (pData->provisioned) {
	         log("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);

	         _my_address = pData->address;

	         enable_sound_interrupts();
	         node_init();

	         // Initialize Low Power Node functionality
	         lpn_init();

	         DI_Print("provisioned", DI_ROW_STATUS);
	       } else {
	         log("node is unprovisioned\r\n");
	         DI_Print("unprovisioned", DI_ROW_STATUS);

	         log("starting unprovisioned beaconing...\r\n");
	         // Enable ADV and GATT provisioning bearer
	         gecko_cmd_mesh_node_start_unprov_beaconing(PB_ADV | PB_GATT);
	       }
	       break;

	     case gecko_evt_system_external_signal_id:


	    	if ((pEvt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_NOISE ))			//Ext. event set by noise sensor interrupt handler
	    	{
	    		    log("NOISE DETECTED\n\r");
	    		    change_switch_position(ON);

	    	}
	    	else if(pEvt->data.evt_system_external_signal.extsignals & I2C_SCHEDULE)
	    	{
	    		//SLEEP_SleepBlockEnd(2);
	    		NVIC_DisableIRQ(I2C0_IRQn);
	    	}
	    	else if(pEvt->data.evt_system_external_signal.extsignals & LETIMER_COMP1_SCHEDULE) //Ext. event set by comp1 interrupt handler
	    	{
	    		SLEEP_SleepBlockEnd(2);
	    		LETIMER_IntDisable(LETIMER0,LETIMER_IFC_COMP1);
	    	}
	    	else if(pEvt->data.evt_system_external_signal.extsignals & LETIMER_UF_SCHEDULE) //Ext. event set by UF interrupt handler
	    	{
	    		SLEEP_SleepBlockEnd(2);
	    		TempReadSequence();
	    	}



	     break;
	     case gecko_evt_mesh_node_provisioning_started_id:
	       log("Started provisioning\r\n");
	       DI_Print("provisioning...", DI_ROW_STATUS);

	       // start timer for blinking LEDs to indicate which node is being provisioned
	       gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(250),
	                                         PROVISIONING_TIMER,
	                                         REPEATING);
	       break;

	     case gecko_evt_mesh_node_provisioned_id:
	       provisioning_finished = 1;
	       node_init();
	       // try to initialize lpn after 30 seconds, if no configuration messages come
	       set_configuration_timer(30000);
	       log("node provisioned, got address=%x\r\n", pEvt->data.evt_mesh_node_provisioned.address);
	       // stop LED blinking when provisioning complete
	       gecko_cmd_hardware_set_soft_timer(TIMER_STOP,
	                                         PROVISIONING_TIMER,
	                                         REPEATING);
	       //led_set_state(LED_STATE_OFF);
	       DI_Print("provisioned", DI_ROW_STATUS);


	       enable_sound_interrupts();
			#if TEMP_SENSOR
	         	 //selectedFREQ = selectLXFO();
	         	 //initialize_LETIMER0(selectedFREQ);
	         	//i2cStructInit();
			#endif
	       break;

	     case gecko_evt_mesh_node_provisioning_failed_id:
	       log("provisioning failed, code 0x%x\r\n", pEvt->data.evt_mesh_node_provisioning_failed.result);
	       DI_Print("prov failed", DI_ROW_STATUS);
	       /* start a one-shot timer that will trigger soft reset after small delay */
	       gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
	                                         RESTART_TIMER,
	                                         SINGLE_SHOT);
	       break;

	     case gecko_evt_mesh_node_key_added_id:
	       log("got new %s key with index 0x%x\r\n",
	           pEvt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
	           pEvt->data.evt_mesh_node_key_added.index);
	       // try to init lpn 5 seconds after adding key
	       set_configuration_timer(5000);
	       break;

	     case gecko_evt_mesh_node_model_config_changed_id:
	       log("model config changed\r\n");
	       // try to init lpn 5 seconds after configuration change
	       set_configuration_timer(5000);
	       break;

	     case gecko_evt_mesh_node_config_set_id:
	       log("model config set\r\n");
	       // try to init lpn 5 seconds after configuration set
	       set_configuration_timer(5000);
	       break;

	     case gecko_evt_le_connection_opened_id:
	       log("evt:gecko_evt_le_connection_opened_id\r\n");
	       num_connections++;
	       conn_handle = pEvt->data.evt_le_connection_opened.connection;
	       DI_Print("connected", DI_ROW_CONNECTION);
	       break;

	     case gecko_evt_le_connection_closed_id:
	       /* Check if need to boot to dfu mode */
	       if (boot_to_dfu) {
	         /* Enter to DFU OTA mode */
	         gecko_cmd_system_reset(2);
	       }

	       log("evt:conn closed, reason 0x%x\r\n", pEvt->data.evt_le_connection_closed.reason);
	       conn_handle = 0xFF;
	       if (num_connections > 0) {
	         if (--num_connections == 0) {
	           DI_Print("", DI_ROW_CONNECTION);
	         }
	       }
	       break;

	     case gecko_evt_mesh_node_reset_id:
	       log("evt gecko_evt_mesh_node_reset_id\r\n");
	       initiate_factory_reset();
	       break;

	     case gecko_evt_le_connection_parameters_id:
	       log("connection params: interval %d, timeout %d\r\n",
	           pEvt->data.evt_le_connection_parameters.interval,
	           pEvt->data.evt_le_connection_parameters.timeout);
	       break;

	     case gecko_evt_le_gap_adv_timeout_id:
	       // these events silently discarded
	       break;

	     case gecko_evt_gatt_server_user_write_request_id:
	       if (pEvt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
	         /* Set flag to enter to OTA mode */
	         boot_to_dfu = 1;
	         /* Send response to Write Request */
	         gecko_cmd_gatt_server_send_user_write_response(
	           pEvt->data.evt_gatt_server_user_write_request.connection,
	           gattdb_ota_control,
	           bg_err_success);

	         /* Close connection to enter to DFU OTA mode */
	         gecko_cmd_le_connection_close(pEvt->data.evt_gatt_server_user_write_request.connection);
	       }
	       break;

	     case gecko_evt_mesh_proxy_connected_id:
	     case gecko_evt_mesh_proxy_disconnected_id:
	       handle_mesh_proxy_events(pEvt);
	       break;

	     case gecko_evt_mesh_lpn_friendship_established_id:
	    	 //send_level_request(a1,d1);
	    	 //break;
	     case gecko_evt_mesh_lpn_friendship_failed_id:
	     case gecko_evt_mesh_lpn_friendship_terminated_id:
	       handle_lpn_events(pEvt);


	       break;
	     //case gecko_evt_hardware_soft_timer_id:
	    //	 break;
	     default:
	       //log("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
	       break;
	   //}
	 }

	 /** @} (end addtogroup app) */
	 /** @} (end addtogroup Application) */


#endif //NOISE_SENSOR

#if PIR_SENSOR == 1
		  uint16_t result;
		   char buf[30];

		   if (NULL == pEvt) {
		     return;
		   }

		   switch (evt_id) {
		     case gecko_evt_system_boot_id:
		       // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
		       if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0
		           ) {
		         initiate_factory_reset();
		       } else {
		         struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

		         set_device_name(&pAddr->address);

		         // Initialize Mesh stack in Node operation mode, it will generate initialized event
		         result = gecko_cmd_mesh_node_init()->result;
		         if (result) {
		           sprintf(buf, "init failed (0x%x)", result);
		           DI_Print(buf, DI_ROW_STATUS);
		         }
		       }
		       break;

		     case gecko_evt_hardware_soft_timer_id:
		       switch (pEvt->data.evt_hardware_soft_timer.handle) {
		         case FACTORY_RESET_TIMER:
		           // reset the device to finish factory reset
		           gecko_cmd_system_reset(0);
		           break;

		         case RESTART_TIMER:
		           // restart timer expires, reset the device
		           gecko_cmd_system_reset(0);
		           break;

		         case PROVISIONING_TIMER:
		           // toggle LED to indicate the provisioning state
		           if (!provisioning_finished) {
		             //led_set_state(LED_STATE_PROV);
		           }
		           break;

		         case RETRANS_ONOFF_TIMER:
		           handle_retrans_timer_evt(pEvt);
		           break;

		         case NODE_CONFIGURED_TIMER:
		         case FRIEND_FIND_TIMER:
		           handle_lpn_timer_evt(pEvt);
		           break;

		         default:
		           break;
		       }

		       break;

		     case gecko_evt_mesh_node_initialized_id:
		       log("node initialized\r\n");

		       // Initialize generic client models
		       result = gecko_cmd_mesh_generic_client_init_on_off()->result;
		       if (result) {
		         log("mesh_generic_client_init_on_off failed, code 0x%x\r\n", result);
		       }

		       result = gecko_cmd_mesh_generic_client_init_common()->result;
		       if (result) {
		         log("mesh_generic_client_init_common failed, code 0x%x\r\n", result);
		       }



		       struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(pEvt->data);

		       if (pData->provisioned) {
		         log("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);

		         _my_address = pData->address;

		         pir_init();
		         node_init();

		         // Initialize Low Power Node functionality
		         lpn_init();

		         DI_Print("provisioned", DI_ROW_STATUS);
		       } else {
		         log("node is unprovisioned\r\n");
		         DI_Print("unprovisioned", DI_ROW_STATUS);

		         log("starting unprovisioned beaconing...\r\n");
		         // Enable ADV and GATT provisioning bearer
		         gecko_cmd_mesh_node_start_unprov_beaconing(PB_ADV | PB_GATT);
		       }
		       break;

		     case gecko_evt_system_external_signal_id:

			    	if ((pEvt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PIR) ==EXT_SIGNAL_PIR)
			    	    			 {
			    	    		log("PIR external event_ #1");
			    	    		gpioLedPIRSetOn();
			    	    		change_switch_position(ON);
			    	    			 }


		     break;
		     case gecko_evt_mesh_node_provisioning_started_id:
		       log("Started provisioning\r\n");
		       DI_Print("provisioning...", DI_ROW_STATUS);

		       // start timer for blinking LEDs to indicate which node is being provisioned
		       gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(250),
		                                         PROVISIONING_TIMER,
		                                         REPEATING);
		       break;

		     case gecko_evt_mesh_node_provisioned_id:
		       provisioning_finished = 1;
		       node_init();
		       // try to initialize lpn after 30 seconds, if no configuration messages come
		       set_configuration_timer(30000);
		       log("node provisioned, got address=%x\r\n", pEvt->data.evt_mesh_node_provisioned.address);
		       // stop LED blinking when provisioning complete
		       gecko_cmd_hardware_set_soft_timer(TIMER_STOP,
		                                         PROVISIONING_TIMER,
		                                         REPEATING);
		       //led_set_state(LED_STATE_OFF);
		       DI_Print("provisioned", DI_ROW_STATUS);


		       pir_init();
		       break;

		     case gecko_evt_mesh_node_provisioning_failed_id:
		       log("provisioning failed, code 0x%x\r\n", pEvt->data.evt_mesh_node_provisioning_failed.result);
		       DI_Print("prov failed", DI_ROW_STATUS);
		       /* start a one-shot timer that will trigger soft reset after small delay */
		       gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
		                                         RESTART_TIMER,
		                                         SINGLE_SHOT);
		       break;

		     case gecko_evt_mesh_node_key_added_id:
		       log("got new %s key with index 0x%x\r\n",
		           pEvt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
		           pEvt->data.evt_mesh_node_key_added.index);
		       // try to init lpn 5 seconds after adding key
		       set_configuration_timer(5000);
		       break;

		     case gecko_evt_mesh_node_model_config_changed_id:
		       log("model config changed\r\n");
		       // try to init lpn 5 seconds after configuration change
		       set_configuration_timer(5000);
		       break;

		     case gecko_evt_mesh_node_config_set_id:
		       log("model config set\r\n");
		       // try to init lpn 5 seconds after configuration set
		       set_configuration_timer(5000);
		       break;

		     case gecko_evt_le_connection_opened_id:
		       log("evt:gecko_evt_le_connection_opened_id\r\n");
		       num_connections++;
		       conn_handle = pEvt->data.evt_le_connection_opened.connection;
		       DI_Print("connected", DI_ROW_CONNECTION);
		       break;

		     case gecko_evt_le_connection_closed_id:
		       /* Check if need to boot to dfu mode */
		       if (boot_to_dfu) {
		         /* Enter to DFU OTA mode */
		         gecko_cmd_system_reset(2);
		       }

		       log("evt:conn closed, reason 0x%x\r\n", pEvt->data.evt_le_connection_closed.reason);
		       conn_handle = 0xFF;
		       if (num_connections > 0) {
		         if (--num_connections == 0) {
		           DI_Print("", DI_ROW_CONNECTION);
		         }
		       }
		       break;

		     case gecko_evt_mesh_node_reset_id:
		       log("evt gecko_evt_mesh_node_reset_id\r\n");
		       initiate_factory_reset();
		       break;

		     case gecko_evt_le_connection_parameters_id:
		       log("connection params: interval %d, timeout %d\r\n",
		           pEvt->data.evt_le_connection_parameters.interval,
		           pEvt->data.evt_le_connection_parameters.timeout);
		       break;

		     case gecko_evt_le_gap_adv_timeout_id:
		       // these events silently discarded
		       break;

		     case gecko_evt_gatt_server_user_write_request_id:
		       if (pEvt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
		         /* Set flag to enter to OTA mode */
		         boot_to_dfu = 1;
		         /* Send response to Write Request */
		         gecko_cmd_gatt_server_send_user_write_response(
		           pEvt->data.evt_gatt_server_user_write_request.connection,
		           gattdb_ota_control,
		           bg_err_success);

		         /* Close connection to enter to DFU OTA mode */
		         gecko_cmd_le_connection_close(pEvt->data.evt_gatt_server_user_write_request.connection);
		       }
		       break;

		     case gecko_evt_mesh_proxy_connected_id:
		     case gecko_evt_mesh_proxy_disconnected_id:
		       handle_mesh_proxy_events(pEvt);
		       break;

		     case gecko_evt_mesh_lpn_friendship_established_id:
		     case gecko_evt_mesh_lpn_friendship_failed_id:
		     case gecko_evt_mesh_lpn_friendship_terminated_id:
		       handle_lpn_events(pEvt);
		       DI_Print("LPN with friend", 4);                     // LPN has established friendship with the friend

		       break;

		     default:
		       //log("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
		       break;
		   //}
		 }

		 /** @} (end addtogroup app) */
		 /** @} (end addtogroup Application) */


#endif //PIR SENSOR


} // handle_gecko_event()



