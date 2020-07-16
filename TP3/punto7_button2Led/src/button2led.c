/*
 * button2led.c
 */

#include "button2led.h"

int main( void ) {

	inicializar_plataforma();
	inicializar_buttonState();

	// De examples/c/sapi/finite_state_machine/fsm_debounce/fsm_deb_debounce_gtoggle
	delay_t actualizar_boton;
	delayInit(&actualizar_boton, 50);

	// al agregar el bool quito la logica de rising y falling.
	bool_t transicion = FALSE;

	while(TRUE) {

	      // Actualizo MEF Boton cada tiempo actualizarMefBoton
	      if( delayRead(&actualizar_boton) ) {

	         my_ButtonUpdate(MY_GPIO_TEC1, &transicion);
	         if(transicion == TRUE) {
	        	my_PrintChar(MY_GPIO_TEC1, button_state_tecla_1);
	         }

	         my_ButtonUpdate(MY_GPIO_TEC2, &transicion);
	         if(transicion == TRUE) {
	        	my_PrintChar(MY_GPIO_TEC2, button_state_tecla_2);
	         }

	         my_ButtonUpdate(MY_GPIO_TEC3, &transicion);
	         if(transicion == TRUE) {
	        	my_PrintChar(MY_GPIO_TEC3, button_state_tecla_3);
	         }

	         my_ButtonUpdate(MY_GPIO_TEC4, &transicion);
	         if(transicion == TRUE) {
	        	my_PrintChar(MY_GPIO_TEC4, button_state_tecla_4);
	         }

	      }






	   }

	return 0;
}

void inicializar_plataforma() {
	// Inicializacion y configuracion de la plataforma.
	boardConfig();

	Board_Init();
	SystemCoreClockUpdate();

	// Inicializacion y configuracion de la UART_USB con Callback.
	uartConfig(UART_232,115200);
	uartCallbackSet(UART_232, UART_RECEIVE, onRx, NULL);
	uartInterrupt(UART_232, true);
}

void inicializar_buttonState() {

	my_ButtonState_t button_state_tecla_1;
	my_ButtonState_t button_state_tecla_2;
	my_ButtonState_t button_state_tecla_3;
	my_ButtonState_t button_state_tecla_4;

	button_state_tecla_1 = buttonInit();
	button_state_tecla_2 = buttonInit();
	button_state_tecla_3 = buttonInit();
	button_state_tecla_4 = buttonInit();
}


// de examples/c/sapi/uart/rx_interrupt
void onRx(void* noUsado) {
	char caracter_recibido = uartRxRead(UART_232);
	switch(caracter_recibido) {
		case '1':
			my_gpio_write(MY_GPIO_LEDR, 1);
			break;
		case '2':
			my_gpio_write(MY_GPIO_LEDG, 1);
			break;
		case '3':
			my_gpio_write(MY_GPIO_LEDB, 0);
			break;
		case '4':
			my_gpio_write(MY_GPIO_LED1, 0);
			break;
	}
}

//de sapi_print.hc
void my_PrintChar(gpioMap_t tecla, ButtonState_t ButtonState){

	switch (tecla){

		case MY_GPIO_TEC1:
			 my_gpio_write(MY_GPIO_LEDR, HIGH);
			 uartWriteByte( UART_232, '1' );
			 break;

		case MY_GPIO_TEC2:
			my_gpio_write(MY_GPIO_LEDG,HIGH);
			uartWriteByte( UART_232, '2' );
			break;

		case MY_GPIO_TEC3:
			my_gpio_write(MY_GPIO_LEDB,LOW);
			uartWriteByte( UART_232, '3' );
			break;

		case MY_GPIO_TEC4:
			my_gpio_write(MY_GPIO_LED1, LOW);
			uartWriteByte(UART_232, '4');
			break;

		default:
			stdioPrintf("Error. \n");
			break;
	}
}








