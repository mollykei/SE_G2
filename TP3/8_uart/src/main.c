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

#include "main.h"
#include "Uart_8.h"
#include "sapi.h"
#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"
#include "sapi_uart.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*! This is a state machine */
static Uart_8 statechart;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void uart_8Iface_opTx(const Uart_8* handle, const sc_integer send, const sc_integer uart)
{
	if(uart == UART_8_UART_8IFACE_UARTUSB) {
	  //uartSetPendingInterrupt(UART_USB);
		uartWriteByte( UART_USB, send );

	} else if(uart == UART_8_UART_8IFACE_UART3) {
	  //uartSetPendingInterrupt(UART_232);
		uartWriteByte( UART_232, send );

	}
}


void uart_8Iface_opLed(const Uart_8* handle)
{
	// Leo la variable que recibi por UART
	sc_integer rxChar = uart_8Iface_get_viRxChar(handle);

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

volatile bool_t rxUartUSBFlag = false;
volatile bool_t rxUart3Flag = false;
volatile bool_t dataToSendToPcPending = false;
volatile bool_t txUart3Pending = false;
volatile uint8_t datoUSB = 0;
volatile uint8_t datoUart3 = 0;

void onRxUartUSB( void *noUsado )
{
	rxUartUSBFlag = true;
	txUart3Pending = true;
	datoUSB = uartRxRead(UART_USB);
}

void onTxUartUSB( void *noUsado )
{
	sc_integer value = uart_8Iface_get_viRxChar(&statechart);

	if(dataToSendToPcPending) {
		dataToSendToPcPending = false;
		uartTxWrite(UART_USB, value);
	}
}

void onRxUart3( void *noUsado )
{
	dataToSendToPcPending = true;
	rxUart3Flag = true;
	datoUart3 = uartRxRead(UART_232);
}

void onTxUart3( void *noUsado )
{
	sc_integer value = uart_8Iface_get_viRxChar(&statechart);

	if(txUart3Pending) {
		txUart3Pending = false;
		uartTxWrite(UART_232, value);
	}
}

int main(void)
{
	/* Board config*/
	boardConfig();

	/*Configuro el Debug UART*/
	uartConfig(UART_USB, 115200);
	uartCallbackSet(UART_USB, UART_RECEIVE, onRxUartUSB, NULL);
	//uartCallbackSet(UART_USB, UART_TRANSMITER_FREE, onTxUartUSB, NULL);
	uartInterrupt(UART_USB, true);

	/* Inicializar la UART_232 */
	uartConfig(UART_232, 115200);
	uartCallbackSet(UART_232, UART_RECEIVE, onRxUart3, NULL);
	//uartCallbackSet(UART_232, UART_TRANSMITER_FREE, onTxUart3, NULL);
	uartInterrupt(UART_232, true);

	/* Statechart Initialization */
	uart_8_init(&statechart);
	uart_8_enter(&statechart);

	while (1) {
		// Si me llego una interrupción del uart
		if(rxUartUSBFlag) {
			// reseteo el flag
			rxUartUSBFlag = false;
			// recibí data por uart
			uart_8Iface_set_viRxChar(&statechart, datoUSB);
			uart_8Iface_raise_evUartUsbRx(&statechart);
		}
		if(rxUart3Flag) {
			rxUart3Flag = false;
			uart_8Iface_set_viRxChar(&statechart, datoUart3);
			uart_8Iface_raise_evUart3Rx(&statechart);

		}
		uart_8_runCycle(&statechart);
	}
}


/*==================[end of file]============================================*/
