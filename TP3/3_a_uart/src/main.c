/* Copyright 2017, Pablo Ridolfi, Juan Manuel Cruz
 * All rights reserved.
 *
 * This file is part of Workspace.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @brief This is a simple statechart example using Yakindu Statechart Tool
 * Plug-in (update site: http://updates.yakindu.org/sct/mars/releases/).
 */

/** \addtogroup statechart Simple UML Statechart example.
 ** @{ */

/*==================[inclusions]=============================================*/

#include "../../3_a_uart/inc/main.h"

#include "../../3_a_uart/gen/Uart.h"
#include "sapi.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*! This is a state machine */
static Uart statechart;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/


/** state machine user-defined external function (action)
 *
 * @param handle es un puntero a la estructura que contene la máquina de estados
 * @param uartRxCharacter es el caracter que se recibió por UART
 * @return nada
 */

void uartIface_opLed(const Uart* handle)
{
	// Leo la variable que recibi por UART
	sc_integer rxChar = uartIface_get_viUartRx(handle);

	switch(rxChar) {

	case MY_LED1_ON:
		gpioWrite(LED1, ON);
		break;
	case MY_LED2_ON:
		gpioWrite(LED2, ON);
		break;
	case MY_LED3_ON:
		gpioWrite(LED3, ON);
		break;
	case MY_LED1_OFF:
		gpioWrite(LED1, OFF);
		break;
	case MY_LED2_OFF:
		gpioWrite(LED2, OFF);
		break;
	case MY_LED3_OFF:
		gpioWrite(LED3, OFF);
		break;
	default:
		;
		//"ERROR: Caracter no válido"
	}

}


int main(void)
{
	/* Board config*/
	boardConfig();

	/*Configuro el Debug UART*/
	uartConfig( UART_USB, 115200 );

	/* Statechart Initialization */
	uart_init(&statechart);
	uart_enter(&statechart);

	uint8_t dato = 0;

	while (1) {

		// Esta peracion es bloqueante
		if(uartReadByte(UART_USB, &dato)) {
			// recibí data por uart
			uartIface_raise_evUartRx(&statechart);
			uartIface_set_viUartRx(&statechart, dato);
		}
		uart_runCycle(&statechart);

	}
}


/*==================[end of file]============================================*/
