/**************************************************************************************************
 * @file      irq.c
 * @version   0.0.1
 * @brief     irq.c interrupt functions
 *
 * @author    Aaksha Jaywant, aaksha.jaywant@colorado.edu
 * @date      Sep 6, 2020
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor  David Sluiter
 *
 * @assignment ecen5823-assignment4-aakshajaywant
 * @due        Sep 25, 2020
 *
 * @resources  Utilized Silicon Labs' EMLIB peripheral libraries to implement functionality.
 *         		- system_efr32.h used for interrupt handler
 *				- https://www.silabs.com/content/usergenerated/asi/cloud/attachments/siliconlabs/en/community/mcu/32-bit/forum/jcr:content/content/primary/qna/letimer_for_low_ener-4b0X/LETimerSourceCode.c
 *
 * @copyright  All rights reserved. Distribution allowed only for the use of assignment grading.
 *       Use of code excerpts allowed at the discretion of author. Contact for permission.
 */


#include "irq.h"

uint32_t countVAL=0;


/*
* FUNCTION:- LETIMER0_IRQHandler
* DESCRIPTION:- It is the LETIMER0 interrupt handler which sets the UF or COMP1 Event
* PARAMETERS:- None
* RETURN VALUE:- None
**/



void LETIMER0_IRQHandler(void)
{
	CORE_DECLARE_IRQ_STATE;				//Critical section macro

	if ((LETIMER_IntGet(LETIMER0) & LETIMER_IF_UF))										//Underflow Interrupt Flag Check
	{
		log("In timer UF\n\r");
		 TimestampCNT++;
		 LETIMER_IntClear(LETIMER0,(LETIMER_IntGet(LETIMER0)));
		 CORE_ENTER_CRITICAL();
		 schedulerSetEventUF();													//Call to the UF flag event scheduler
		 CORE_EXIT_CRITICAL();
										//clear interrupts

	}
else if ((LETIMER_IntGet(LETIMER0) & LETIMER_IF_COMP1))
	{
	log("In timer COMP1\n\r");
		LETIMER_IntClear(LETIMER0, (LETIMER_IntGet(LETIMER0)));
		CORE_ENTER_CRITICAL();
		schedulerSetEventCOMP1();													//Call to the UF flag event scheduler
		CORE_EXIT_CRITICAL();
	}
}


/*
* FUNCTION:- I2C0_IRQHandler
* DESCRIPTION:- It is an I2C interrupt handler which sets the I2C Event
* PARAMETERS:- None
* RETURN VALUE:- None
**/

void I2C0_IRQHandler(void)
{
	CORE_DECLARE_IRQ_STATE;
	log("In i2c int handler\n\r");
	retSTAT = I2C_Transfer(I2C0);
	CORE_ENTER_CRITICAL();
	schedulerSetEventI2C();
	CORE_EXIT_CRITICAL();
}


