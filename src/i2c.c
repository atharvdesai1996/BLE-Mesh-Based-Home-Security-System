/**************************************************************************************************
 * @file      i2c.c
 * @version   0.0.1
 * @brief     i2c.c includes i2c structure initialize, write, read, sensor enable & disable and temperature print functions
 *
 * @author    Aaksha Jaywant, aaksha.jaywant@colorado.edu
 * @date      Sep 15, 2020
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor  David Sluiter
 *
 * @assignment ecen5823-assignment4-aakshajaywant
 * @due        Sep 25, 2020
 *
 * @resources  Utilized Silicon Labs' EMLIB peripheral libraries to implement functionality.
 *         		1) https://www.silabs.com/community/mcu/32-bit/forum.topic.html/i2c_pin_mapping_loca-ixJK
 * 				2) Lec8 Scheduler Guidance
 * 				3) i2cspm.h used for i2c and Si7021 communication
 * 				4) core_cm4.h used for I2C NVIC enable & disable
 *
 * @copyright  All rights reserved. Distribution allowed only for the use of assignment grading.
 *       Use of code excerpts allowed at the discretion of author. Contact for permission.
 */



#include "i2c.h"


/*
* FUNCTION:- TempReadSequence
* DESCRIPTION:- This function takes the temperature reading
* PARAMETERS:- None
* RETURN VALUE:- None
**/

void TempReadSequence(void)
{


	//enableSensor();
	//timerWaitUs(80000);
	I2CWriteForIRQ();
//	timerWaitUs(10000);
//	I2CReadForIRQ();
//	I2CTempPrint();
	//disableSensor();
}

/*
* FUNCTION:- i2cWrite
* DESCRIPTION:- This function writes the command register to read
* PARAMETERS:- ipData (It is the command register address), len (length of the write buffer in bytes)
* RETURN VALUE:- None
**/

void i2cWrite(uint8_t ipData, uint8_t len)
{
		i2cWrDATA[0] = ipData;
		transferINIT.addr = tempADDRESS;
		transferINIT.flags = I2C_FLAG_WRITE;
		transferINIT.buf[0].data = (uint8_t *)i2cWrDATA;
		transferINIT.buf[0].len = len;
		retSTAT = I2CSPM_Transfer(I2C0,&transferINIT);
		if(retSTAT != i2cTransferDone)
		{
			log("Failed to write %d byte, return value was %d\n\r",len,retSTAT);
		}
}


/*
* FUNCTION:- i2cRead
* DESCRIPTION:- This function read the temp value from the slave
* PARAMETERS:- lenRead (length of the read buffer in bytes)
* RETURN VALUE:- None
**/

void i2cRead(uint8_t lenRead)
{

		uint8_t *dataREAD = malloc(lenRead * sizeof(uint8_t));
		transferINIT1.addr = tempADDRESS;
		transferINIT1.flags = I2C_FLAG_READ;
		transferINIT1.buf[0].data = dataREAD;
		transferINIT1.buf[0].len = lenRead;
		retSTAT = I2CSPM_Transfer(I2C0,&transferINIT1);
		if(retSTAT != i2cTransferDone)
		{
			log("Failed to read %d bytes, return value was %d\n\r",lenRead,retSTAT);
		}
		tempVAL = (((uint16_t)(*dataREAD)) << 8) | *(dataREAD+1);
		tempFINAL = ((float)((float)(175.72 * tempVAL))/65536)-46.85;

		log("Read temperature %0.2f\n\r", tempFINAL);
		free(dataREAD);

}

/*
* FUNCTION:- enableSensor
* DESCRIPTION:- This function enables the I2C sensor enable pin
* PARAMETERS:- None
* RETURN VALUE:- None
**/

void enableSensor(void)
{
	GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 1);
}

/*
* FUNCTION:- disableSensor
* DESCRIPTION:- This function disables the I2C sensor disable pin
* PARAMETERS:- None
* RETURN VALUE:- None
**/
void disableSensor(void)
{

	GPIO_PinOutClear(gpioPortD,15);
}

/*
* FUNCTION:- i2cStructInit
* DESCRIPTION:- This function initializes the I2CSPM_Init_TypeDef structure
* PARAMETERS:- None
* RETURN VALUE:- None
**/
void i2cStructInit(void)
{
	I2CSPM_Init_TypeDef i2c_obj=
		{
				I2C0,
				gpioPortC,
				10,
				gpioPortC,
				11,
				14,
				16,
				0,
				I2C_FREQ_STANDARD_MAX,
				i2cClockHLRStandard,
		};

	I2CSPM_Init(&i2c_obj);
}

/*
* FUNCTION:- I2CWriteForIRQ
* DESCRIPTION:- This function writes the address of command register to be read, enables the interrupts and starts transfer
* PARAMETERS:- None
* RETURN VALUE:- retSTAT (Return status of the I2C_TransferInit function)
**/

I2C_TransferReturn_TypeDef I2CWriteForIRQ(void)
{
	i2cWrDATA[0] = 0xF3;
	transferINIT.addr = tempADDRESS;
	transferINIT.flags = I2C_FLAG_WRITE;
	transferINIT.buf[0].data = (uint8_t *)i2cWrDATA;
	transferINIT.buf[0].len = 1;
	NVIC_EnableIRQ(I2C0_IRQn);
	SLEEP_SleepBlockBegin(2);
	retSTAT = I2C_TransferInit(I2C0,&transferINIT);
	return retSTAT;
}

/*
* FUNCTION:- I2CReadForIRQ
* DESCRIPTION:- This function starts a temperature read transaction, enables the interrupts and starts transfer
* PARAMETERS:- None
* RETURN VALUE:- retSTAT (Return status of the I2C_TransferInit function)
**/

I2C_TransferReturn_TypeDef I2CReadForIRQ(void)
{
	transferINIT1.addr = tempADDRESS;
	transferINIT1.flags = I2C_FLAG_READ;
	transferINIT1.buf[0].data = i2cRdDATA;				//Read func buffers are different
	transferINIT1.buf[0].len = 2;
	NVIC_EnableIRQ(I2C0_IRQn);
	SLEEP_SleepBlockBegin(2);
	retSTAT = I2C_TransferInit(I2C0,&transferINIT1);
	return retSTAT;
}

/*
* FUNCTION:- I2CTempPrint
* DESCRIPTION:- This function computes the celsius value and logs it on the terminal
* PARAMETERS:- None
* RETURN VALUE:- None
**/

float I2CTempPrint(void)
{
	uint16_t temp_send;
		tempVAL = (((uint16_t)(i2cRdDATA[0])) << 8) | (i2cRdDATA[1]);
		tempFINAL = ((float)((float)(175.72 * tempVAL))/65536)-46.85;
		temp_send = ((175.72 * tempVAL)/65536)-46.85;
		log("Read temperature %lu\n\r", (uint32_t)tempFINAL);
		send_level_request(temp_send,0);
		return tempFINAL;
}

