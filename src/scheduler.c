/**************************************************************************************************
 * @file      scheduler.c
 * @version   0.0.1
 * @brief     scheduler.c includes the scheduler set event, get event, process event and state machine functions
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



/*
 * scheduler.c
 *
 *  Created on: 14-Sep-2020
 *      Author: Aaksha Jaywant
 */

//REFERENCE:- https://www.geeksforgeeks.org/find-position-of-the-only-set-bit/
#include "scheduler.h"




uint32_t UF_flag = 0;								//global flag for scheduler


/*
* FUNCTION:- schedulerSetEventUF
* DESCRIPTION:- This function ORs the Underflow IF register bit position with the global variable
* PARAMETERS:- None
* RETURN VALUE:- None
**/
void schedulerSetEventUF(void)
{

	//UF_flag |= LETIMER_IF_UF;
	 gecko_external_signal(LETIMER_UF_SCHEDULE);
}


/*
* FUNCTION:- schedulerSetEventComp1
* DESCRIPTION:- This function ORs the Comp1 IF register bit position with the global variable
* PARAMETERS:- None
* RETURN VALUE:- None
**/
void schedulerSetEventCOMP1(void)
{
	//UF_flag |= LETIMER_IF_COMP1;
	gecko_external_signal(LETIMER_COMP1_SCHEDULE);
}


/*
* FUNCTION:- schedulerSetEventI2C
* DESCRIPTION:- This function ORs the I2C event set bit with the global variable
* PARAMETERS:- None
* RETURN VALUE:- None
**/
void schedulerSetEventI2C(void)
{
	//UF_flag |= 0x00000001;
	gecko_external_signal(I2C_SCHEDULE);
}

/*
* FUNCTION:- get_event
* DESCRIPTION:- This function checks which interrupt has occurred
* PARAMETERS:- None
* RETURN VALUE:- theEvent
**/

uint32_t get_event()
{
	CORE_DECLARE_IRQ_STATE;
	uint32_t theEvent = 1, i=1;
	CORE_ENTER_CRITICAL();
	while (!(i & UF_flag))						//checks at which position of IF register the bit is set
	{
	   i = i << 1;
	   ++theEvent;
	}
	UF_flag &= !i;								//clear UF flag
	CORE_EXIT_CRITICAL();
	return theEvent;
}

/*
* FUNCTION:- events_present
* DESCRIPTION:- This function checks if any events are pending
* PARAMETERS:- None
* RETURN VALUE:- true or false
**/

bool events_present()
{
	if(UF_flag == 0)
		return false;
	else
		return true;
}

/*
* FUNCTION:- process_event
* DESCRIPTION:- This function processes the event returned by get_event() function
* PARAMETERS:- eventNum (contains the event returned by get_event() function
* RETURN VALUE:- None
**/


void process_event(uint32_t eventNum)
{
	if(eventNum == 1)
	{
		SLEEP_SleepBlockEnd(2);
		NVIC_DisableIRQ(I2C0_IRQn);
	}
	if(eventNum == 2)
	{
		SLEEP_SleepBlockEnd(4);
		LETIMER_IntDisable(LETIMER0,LETIMER_IFC_COMP1);
	}
	if(eventNum == 3)
	{
		TempReadSequence();
	}
}

/*
* FUNCTION:- state_machine
* DESCRIPTION:- This is an event driven state machine implementation for reading temperature from the sensor
* PARAMETERS:- event (takes the event returned by get_event() function as an input)
* RETURN VALUE:- None
**/


void state_machine(struct gecko_cmd_packet* event)				//changed uint32 event
{

	State_t currentState;
	static State_t nextState = stateIdle;
	currentState = nextState;

	switch(currentState)
	{
	log("IN STATE MACHINE\n\r");
		case stateIdle:
			log("IN STATE1\n\r");
			nextState = stateIdle;

			if((event->data.evt_system_external_signal.extsignals & LETIMER_IF_UF))
			{

				enableSensor();
				nextState = stateTimer80ms;
				timerWaitUs(80000);
				//gecko_cmd_hardware_set_soft_timer(2650,RESTART_TIMER,1);
			}
			break;
		case stateTimer80ms:

			log("80ms 2\n\r");
			nextState = stateTimer80ms;

			if((event->data.evt_system_external_signal.extsignals  & LETIMER_IF_COMP1) == 2 )
			{
				LETIMER_IntDisable(LETIMER0,LETIMER_IFC_COMP1);
				nextState = stateI2CWrite;
				retSTAT = I2CWriteForIRQ();
				if(retSTAT != i2cTransferInProgress)
					log("ERROR I2cWrite");
			}
			break;
		case stateI2CWrite:
			log("IN STATE3\n\r");
			nextState = stateI2CWrite;

			if(((event->data.evt_system_external_signal.extsignals & 0x00000001) == 1) && retSTAT == i2cTransferDone )
			{
				SLEEP_SleepBlockEnd(2);
				NVIC_DisableIRQ(I2C0_IRQn);
				timerWaitUs(10800);
				//gecko_cmd_hardware_set_soft_timer(340,RESTART_TIMER,1);
				nextState = stateTimer10ms;
			}
			break;
		case stateTimer10ms:
			log("IN STATE4\n\r");
			nextState = stateTimer10ms;

			if((event->data.evt_system_external_signal.extsignals & LETIMER_IF_COMP1) == 2 )
			{
				LETIMER_IntDisable(LETIMER0,LETIMER_IFC_COMP1);
				nextState = stateI2CRead;
				retSTAT = I2CReadForIRQ();
				if(retSTAT != i2cTransferInProgress)
					log("ERROR I2cRead");

			}
			break;
		case stateI2CRead:
			nextState = stateI2CRead;

			if(((event->data.evt_system_external_signal.extsignals & 0x00000001) == 1) && retSTAT == i2cTransferDone )
			{
				SLEEP_SleepBlockEnd(2);
				NVIC_DisableIRQ (I2C0_IRQn);
				float retprint = I2CTempPrint();
				//disableSensor();
				//temperatureMeasure((uint32_t)retprint);
				nextState = stateIdle;
			}
			break;
		default:
			break;
	}
}



