/**************************************************************************************************
 * @file      timers.c
 * @version   0.0.1
 * @brief     timers.c configures the LETIMER0, it's interrupts and contains a timerWaitUs function for microsecond delay
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
 * @resources  Utilized Silicon Labs' EMLIB peripheral libraries to implement functionality
 *
 * @copyright  All rights reserved. Distribution allowed only for the use of assignment grading.
 *       Use of code excerpts allowed at the discretion of author. Contact for permission.
 */





#include "timers.h"


LETIMER_Init_TypeDef obj_LETIMER0;

/*
* FUNCTION:- initialize_LETIMER0
* DESCRIPTION:- This function initializes the LETIMER0 and enables the interrupts
* PARAMETERS:- It takes divided frequency from oscillator for period and on time calculation
* RETURN VALUE:- None
**/


void initialize_LETIMER0(uint32_t sFREQ)
{
	uint32_t val_PRD;
	val_PRD = (float)(((PRD*(float)0.001)))/((1/(float)(sFREQ)));				//Calculates ticks for loading into counter
	obj_LETIMER0.enable = false;							///Initialize LETIMER_Init_TypeDef structure members
	obj_LETIMER0.debugRun = false;
	obj_LETIMER0.comp0Top = true;
	obj_LETIMER0.bufTop = false;
	obj_LETIMER0.out0Pol = false;
	obj_LETIMER0.out1Pol = false;
	obj_LETIMER0.topValue = val_PRD;
	obj_LETIMER0.repMode = letimerRepeatFree;
	obj_LETIMER0.ufoa0 = letimerUFOANone;
	obj_LETIMER0.ufoa1 = letimerUFOANone;
	LETIMER_Init(LETIMER0,&obj_LETIMER0);
	LETIMER_CompareSet(LETIMER0,0,val_PRD);
	LETIMER_IntClear(LETIMER0,LETIMER_IFC_UF);										//Clear all interrupt flags
	LETIMER_IntEnable(LETIMER0,LETIMER_IEN_UF);										//Enable Interrupts
	NVIC_EnableIRQ(LETIMER0_IRQn);
	LETIMER_Enable(LETIMER0, true);													//Enable the Timer

}

/*
* FUNCTION:- timerWaitUs
* DESCRIPTION:- This function initializes the provides delay by taking LETIMER count as reference
* PARAMETERS:- It takes delay in microseconds
* RETURN VALUE:- None
**/



 void timerWaitUs(uint32_t us_wait)
 {

	 uint32_t counterVAL, freqVAL, waitTIME, diffTIME, finalCOUNT, compVAL;
	 freqVAL = CMU_ClockFreqGet(cmuClock_LETIMER0);					//clock freq get

	 waitTIME = (float)(((us_wait*(float)0.000001)))/((1/(float)(freqVAL)));	//Convert us_wait into tick count
	 counterVAL = LETIMER_CounterGet(LETIMER0);						//Get instantaneous counter value
	 compVAL = LETIMER_CompareGet(LETIMER0,0);						//Get the comp0 value

	 if(counterVAL > waitTIME)										//if instantaneous counter value is greater than us_wait tick count
	 {
		 diffTIME = counterVAL - waitTIME;
		 finalCOUNT =  diffTIME;
	 }
	 else															//if counter val is less than us_wait tick count and rolls over
	 {
		 diffTIME = waitTIME - counterVAL;
		 finalCOUNT = compVAL - diffTIME;
	 }

	 LETIMER_CompareSet(LETIMER0,1, finalCOUNT);
	 LETIMER_IntClear(LETIMER0,LETIMER_IFC_COMP1);
	 LETIMER_IntEnable(LETIMER0,LETIMER_IEN_COMP1);

 }

 /*
 * FUNCTION:- timerGetRunTimeMilliseconds
 * DESCRIPTION:- This function calculates the time in milliseconds using current counter and comp0 value
 * PARAMETERS:- None
 * RETURN VALUE:- msRetVal (Returns current time in milliseconds)
 **/

 uint32_t timerGetRunTimeMilliseconds(void)
 {
 	uint32_t totalPRD,currVAL, msRetVal,currFREQ;
 	currFREQ = CMU_ClockFreqGet(cmuClock_LETIMER0);
 	totalPRD = LETIMER_CompareGet(LETIMER0,0);						//Get the comp0 top value
 	currVAL = LETIMER_CounterGet(LETIMER0);						//Get instantaneous counter value
 	msRetVal = ((((TimestampCNT * totalPRD) + (totalPRD - currVAL))*1000)/currFREQ);
 	return msRetVal;
 }



