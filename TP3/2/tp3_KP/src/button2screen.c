/*
 * button2led.c
 */

#include "button2screen.h"

int main( void ) {

	inicializar_plataforma();

	// De examples/c/sapi/finite_state_machine/fsm_debounce/fsm_deb_debounce_gtoggle
	delay_t actualizar_boton;
	delayInit(&actualizar_boton, 50);

	// al agregar el bool quito la logica de rising y falling.
	bool_t transicion = FALSE;


	my_ButtonState_t button_state_tecla_1;
	my_ButtonState_t button_state_tecla_2;
	my_ButtonState_t button_state_tecla_3;
	my_ButtonState_t button_state_tecla_4;

	button_state_tecla_1 = my_ButtonInit();
	button_state_tecla_2 = my_ButtonInit();
	button_state_tecla_3 = my_ButtonInit();
	button_state_tecla_4 = my_ButtonInit();

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

void inicializar_plataforma(void) {
	// Inicializacion y configuracion de la plataforma.
	boardConfig();

	Board_Init();
	SystemCoreClockUpdate();

	// Inicializacion y configuracion de la UART_USB con Callback.
	uartConfig(UART_USB,115200);
}

//de sapi_print.hc
void my_PrintChar(gpioMap_t tecla, my_ButtonState_t buttonState){

	switch (tecla){

		case MY_GPIO_TEC1:

			if( buttonState == STATE_BUTTON_UP) {
				my_gpio_write(MY_GPIO_LED1, LOW);
			} else {
				uartWriteByte( UART_USB, 'M' );
				my_gpio_write(MY_GPIO_LED2, HIGH);
			}
			break;

		case MY_GPIO_TEC2:
			if( buttonState == STATE_BUTTON_UP) {
				my_gpio_write(MY_GPIO_LED1, LOW);
			} else {
				uartWriteByte( UART_USB, 'O' );
				my_gpio_write(MY_GPIO_LED2, HIGH);
			}
			break;

		case MY_GPIO_TEC3:
			if( buttonState == STATE_BUTTON_UP) {
				my_gpio_write(MY_GPIO_LED1, LOW);
			} else {
				uartWriteByte( UART_USB, 'L' );
				my_gpio_write(MY_GPIO_LED2, HIGH);
			}
			break;

		case MY_GPIO_TEC4:
			if( buttonState == STATE_BUTTON_UP) {
				my_gpio_write(MY_GPIO_LED1, LOW);
			} else {
				uartWriteByte( UART_USB, 'Y' );
				my_gpio_write(MY_GPIO_LED2, HIGH);
			}
			break;

		default:
			printf("Error. \n");
			break;
	}
}
