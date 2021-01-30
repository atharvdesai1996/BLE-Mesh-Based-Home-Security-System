/**************************************************************************************************
 * @file      oscillators.c
 * @version   0.0.1
 * @brief     oscillators.c include functions to enable the LETIMER0 clock tree
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
 *
 * @copyright  All rights reserved. Distribution allowed only for the use of assignment grading.
 *       Use of code excerpts allowed at the discretion of author. Contact for permission.
 */



#include "oscillators.h"


CMU_LFXOInit_TypeDef lfxoInit;

CMU_ClkDiv_TypeDef div_freq;


/*
* FUNCTION:- selectLXFO
* DESCRIPTION:- This function selects the LXFO oscillator
* PARAMETERS:- None
* RETURN VALUE:- Returns uint32_t divided frequency
**/


uint32_t selectLXFO(void)
{
	uint32_t freq_val;
	CMU_OscillatorEnable(cmuOsc_LFXO ,true,true);					//Osc LFXO
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);				//LFXO clk select
	CMU_ClockEnable(cmuClock_LFA,true);								//LFA clk Enable
	freq_val = CMU_ClockFreqGet(cmuClock_LFA);

	CMU_ClockDivSet(cmuClock_LETIMER0,cmuClkDiv_4);				//clk divide set
	div_freq = CMU_ClockDivGet(cmuClock_LETIMER0);				//divide get
	freq_val = CMU_ClockFreqGet(cmuClock_LETIMER0);					//clock freq get

	CMU_ClockEnable(cmuClock_LETIMER0,true);					//LETIMER0 clk enable
	return freq_val;
}

/*
* FUNCTION:- selectULFRCO
* DESCRIPTION:- This function selects the LXFO oscillator
* PARAMETERS:- None
* RETURN VALUE:- Returns uint32_t divided frequency
**/

uint32_t selectULFRCO(void)
{
	uint32_t freq_val;
	CMU_OscillatorEnable(cmuOsc_ULFRCO ,true,true);				//Osc ULFRCO
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);			//ULFRCO Clk select
	CMU_ClockEnable(cmuClock_LFA,true);								//LFA clk Enable
	freq_val = CMU_ClockFreqGet(cmuClock_LFA);
	CMU_ClockDivSet(cmuClock_LETIMER0,cmuClkDiv_1);				//clk divide set
	div_freq = CMU_ClockDivGet(cmuClock_LETIMER0);				//divide get
	freq_val = CMU_ClockFreqGet(cmuClock_LETIMER0);					//clock freq get
	CMU_ClockEnable(cmuClock_LETIMER0,true);					//LETIMER0 clk enable
	return freq_val;


}


