/***************************************************************************
 * @file  gpio.c was buttons.c from Silabs soc-btmest-switch example code
 * @brief gpio implementation file
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

#include "hal-config.h"
#include "native_gecko.h"
#include <gpiointerrupt.h>
#include <em_rtcc.h>
#include <src/gpio.h>
#include "app_timer.h"




/*******************************************************************************
 * Button initialization. Configure pushbuttons PB0, PB1 as inputs.
 ******************************************************************************/
void button_init(void)
{
  // configure pushbutton PB0 and PB1 as inputs, with pull-up enabled
  GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPull, 1);
  GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInputPull, 1);

}



#if NOISE_SENSOR
/*
* FUNCTION:- sound_init
* DESCRIPTION:- Initializes sound detector sensor pin
* PARAMETERS:- None
* RETURN VALUE:- None
**/
void sound_init(void)
{
  GPIO_PinModeSet(soundPort, soundGate, gpioModeInputPull, 1);

}


/*
* FUNCTION:- sound_interrupt
* DESCRIPTION:- Sound sensor interrupt Handler
* PARAMETERS:- None
* RETURN VALUE:- None
**/
void sound_interrupt(void)
{
	GPIOsound = GPIO_PinInGet(soundPort,soundGate);
			if(GPIOsound == 1)
			{
				log("Noisy\n\r");
				gecko_external_signal(EXT_SIGNAL_NOISE);
			}
}
#endif

/***************************************************************************//**
 * This is a callback function that is invoked each time a GPIO interrupt
 * in one of the pushbutton inputs occurs. Pin number is passed as parameter.
 *
 * @param[in] pin  Pin number where interrupt occurs
 *
 * @note This function is called from ISR context and therefore it is
 *       not possible to call any BGAPI functions directly. The button state
 *       change is signaled to the application using gecko_external_signal()
 *       that will generate an event gecko_evt_system_external_signal_id
 *       which is then handled in the main loop.
 ******************************************************************************/
void button_interrupt(uint8_t pin)
{
    // pin = BSP_BUTTON0_PIN for PB0
	// pin = BSP_BUTTON1_PIN for PB1
	//
	// edit #1
	// Add appropriate code to call gecko_external_signal() for PB0 press
	// and release. See gpio.h for external signal #defines
//#define EXT_SIGNAL_PB0_PRESS             0x01
//#define EXT_SIGNAL_PB0_RELEASE           0x02
//#define EXT_SIGNAL_PB1_PRESS             0x04
//#define EXT_SIGNAL_PB1_RELEASE           0x08

	if(pin == BSP_BUTTON0_PIN){ //PB0 event
		int pin_state = !GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN);
		if(pin_state){ //pressed
			gecko_external_signal(EXT_SIGNAL_PB0_PRESS);
		}
		else{ //released
			gecko_external_signal(EXT_SIGNAL_PB0_RELEASE);
		}
	}

	if(pin == BSP_BUTTON1_PIN){ //PB0 event
		int pin_state = !GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN);
		if(pin_state){ //pressed
			gecko_external_signal(EXT_SIGNAL_PB1_PRESS);
		}
		else{ //released
			gecko_external_signal(EXT_SIGNAL_PB1_RELEASE);
		}
	}




} // button_interrupt()



/*******************************************************************************
 * Enable button interrupts for PB0, PB1. Both GPIOs are configured to trigger
 * an interrupt on the rising and falling edges + enable input glitch filtering.
 * See the main board schematic for how the push buttons are connected.
 * The PB input value behaves this way:
 *
 * GPIO = 1  ---------+                 +-------------------------
 *                    |                 |
 *      = 0           +-----------------+
 *                    |                 |
 *                    v                 v
 *                    pressed           released
 *
 ******************************************************************************/
void enable_button_interrupts(void)
{

    // See: ./platform/emdrv/gpiointerrupt/src/gpiointerrupt.c
	//
    // We can't write our own GPIO_EVEN_IRQHandler() and GPIO_ODD_IRQHandler()
	// routines because these are defined in this file. SiLabs has built an
	// ISR function registration and dispatching system. Don't know why?
	//
	// The calling sequence is:
	//
	//    GPIO_EVEN_IRQHandler() {
	//       iflags = GPIO_IntGetEnabled() & 0x00005555;
	//       GPIO_IntClear(iflags);
	//       GPIOINT_IRQDispatcher(iflags);
	//    }
	//
	//    GPIOINT_IRQDispatcher(iflags) {
	//       calls functions in table gpioCallbacks[ ] array
	//    }
	//
	//    We use GPIOINT_CallbackRegister() to place pointers to functions
	//    (known as callbacks) into the gpioCallbacks[ ] array

  GPIOINT_Init();

  /* configure interrupt for PB0 and PB1, both falling and rising edges */
  GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN,
                    true, true, true);

  GPIO_ExtIntConfig(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, BSP_BUTTON1_PIN,
                   true, true, true);


  /* register the callback function that is invoked when interrupt occurs */
  GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, button_interrupt);
  GPIOINT_CallbackRegister(BSP_BUTTON1_PIN, button_interrupt);

} // enable_button_interrupts()


#if NOISE_SENSOR == 1
/*
* FUNCTION:- enable_sound_interrupts
* DESCRIPTION:- This function enables noise sensor interrupts
* PARAMETERS:- None
* RETURN VALUE:- None
**/
void enable_sound_interrupts(void)
{

    // See: ./platform/emdrv/gpiointerrupt/src/gpiointerrupt.c
	//
    // We can't write our own GPIO_EVEN_IRQHandler() and GPIO_ODD_IRQHandler()
	// routines because these are defined in this file. SiLabs has built an
	// ISR function registration and dispatching system. Don't know why?
	//
	// The calling sequence is:
	//
	//    GPIO_EVEN_IRQHandler() {
	//       iflags = GPIO_IntGetEnabled() & 0x00005555;
	//       GPIO_IntClear(iflags);
	//       GPIOINT_IRQDispatcher(iflags);
	//    }
	//
	//    GPIOINT_IRQDispatcher(iflags) {
	//       calls functions in table gpioCallbacks[ ] array
	//    }
	//
	//    We use GPIOINT_CallbackRegister() to place pointers to functions
	//    (known as callbacks) into the gpioCallbacks[ ] array

  GPIOINT_Init();

  /* configure interrupt for PB0 and PB1, both falling and rising edges */
  GPIO_ExtIntConfig(soundPort, soundGate, soundGate, true, true, true);			//now

  //GPIO_ExtIntConfig(PIRLED_port,PIRLED_pin, PIRLED_pin, true, true, true);

  /* register the callback function that is invoked whenS interrupt occurs */
  GPIOINT_CallbackRegister(soundGate,sound_interrupt);							//now
  //GPIOINT_CallbackRegister(PIRLED_pin,sound_interrupt);

} // enable_button_interrupts()
#endif
/*******************************************************************************
 * GPIO initialization. Configure pushbuttons GPIO PC9 as output for external
 * LED indicator
 ******************************************************************************/
#if PIR_SENSOR == 1
void gpioInit()
{
	GPIO_DriveStrengthSet(PIRLED_port1, gpioDriveStrengthWeakAlternateStrong);
	GPIO_PinModeSet(PIRLED_port1, PIRLED_pin1, gpioModePushPull, false);

}

void gpioLedPIRSetOn()
{
	GPIO_PinOutSet(PIRLED_port1,PIRLED_pin1);
}
void gpioLedPIRSetOff()
{
	GPIO_PinOutClear(PIRLED_port1,PIRLED_pin1);
}

/*******************************************************************************
 * GPIO initialization for PIR Interrupt. Configure pushbuttons GPIO PC8 as input
 * for external PIR interrupt
 *
 * @param[in] None
 *
 ******************************************************************************/
void pir_init()
{
	GPIO_PinModeSet(PIRLED_port, PIRLED_pin, gpioModeInput, 0);            // Setting appropriate pin mode
	GPIO_ExtIntConfig(PIRLED_port,PIRLED_pin, PIRLED_pin, true, true, true);   // configuring the pin as external interrupt
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIOINT_Init();
	GPIOINT_CallbackRegister(PIRLED_pin, pir_interrupt);

}


/***************************************************************************//**
 * This is a callback function that is invoked each time a GPIO interrupt
 * in one of the GPIO based inputs occurs.
 *
 * @param[in] None
 *
 * @note This function is called from ISR context and therefore it is
 *       not possible to call any BGAPI functions directly. The GPIO state
 *       change is signaled to the application using gecko_external_signal()
 *       that will generate an event gecko_evt_system_external_signal_id
 *       which is then handled in the main loop.
 ******************************************************************************/
void pir_interrupt()
{

	if(GPIO_PinInGet(PIRLED_port,PIRLED_pin) == 1)    // Checking of interrupt signal on the configured pin
		{
		printf("motion detected \n\r");

		gecko_external_signal(EXT_SIGNAL_PIR);       // PIR based external event
		}

}
#endif




