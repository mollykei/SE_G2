/*
 * rgb_led_uart.c
 *
 *		Programa basado en /examples/c/sapi/hcsr04_ultrasonic_sensor/ultrasonicSensor
 *		y /examples/c/sapi/pwm/dimmer.
 *
 *		El ejemplo /examples/c/sapi/pwm/rgb_led_uart no funciona. Hay un problema con la inicialización de las interrupciones.
 */

#include "../punto4_rgb_led_uart/rgb_led_uart.h"


// Prototipos
void inicializar_plataforma();
void pwm_configuracion();
void menu_mensajes_brillo();
void menu_mensajes_led();

void brillo_seleccion_modo(uint8_t dato_uart, uint8_t duty_cycle);
void brillo_led_aumentar(uint8_t dato_uart, uint8_t duty_cycle);
void brillo_led_disminuir(uint8_t dato_uart, uint8_t duty_cycle);

int main( void ) {

	// variables del programa
	uint8_t dato_uart = 0;
	uint8_t duty_cycle[3] = {0,0,0};  // LED 1 | LED 2 | LED 3

	// inicializacion de la plataforma
	inicializar_plataforma();

	// pwmInit y pwmWrite son bool_t
	pwm_configuracion();

	// mensajes de programa
	menu_mensajes();

	while(1) {

		seteo_pwm(duty_cycle);

		if(uartReadByte(UART_USB, &dato_uart)) {
			brillo_led(dato_uart, duty_cycle);
		}
	}

	return 0;
}


// Funciones

void inicializar_plataforma() {
	// Inicializacion y configuracion de la plataforma.
	boardConfig();

	Board_Init();
	SystemCoreClockUpdate();

	// Inicializacion y configuracion de la UART_USB.
	uartConfig(UART_USB, 115200);
}

void pwm_configuracion() {

	bool_t pwm_config = 0;

	pwm_config = pwmConfig( 0, PWM_ENABLE );

	pwm_config = pwmConfig( PWM0, PWM_ENABLE_OUTPUT );
	pwm_config = pwmConfig( PWM1, PWM_ENABLE_OUTPUT );
	pwm_config = pwmConfig( PWM2, PWM_ENABLE_OUTPUT );
}

void seteo_pwm(uint8_t duty_cyle) {

	bool_t pwm_set = 0;
	uint8_t pwmVal = 0;

	pwm_set = pwmWrite( PWM0, duty_cyle[0] );
    pwmVal = pwmRead( PWM0 );
    pwmWrite( PWM0, pwmVal );

	pwm_set = pwmWrite( PWM1, duty_cyle[1] );
    pwmVal = pwmRead( PWM1 );
    pwmWrite( PWM1, pwmVal );

	pwm_set = pwmWrite( PWM2, duty_cyle[2] );
    pwmVal = pwmRead( PWM2 );
    pwmWrite( PWM2, pwmVal );
}

void menu_mensajes_brillo() {

	stdioPrintf(UART_USB, "Seleccione una letra para aumentar o disminui el brillo. \n \r");
	stdioPrintf(UART_USB, "AUMENTAR = a \n \r");
	stdioPrintf(UART_USB, "DISMINUIR = d \n \r");
}

void menu_mensajes_led() {

	stdioPrintf(UART_USB, "Presione 1 para modificar brillo del LED1 \n \r");
	stdioPrintf(UART_USB, "Presione 2 para modificar brillo del LED2 \n \r");
	stdioPrintf(UART_USB, "Presione 3 para modificar brillo del LED3 \n \r");
}

void brillo_seleccion_modo(uint8_t dato_uart, uint8_t duty_cycle) {

	switch(dato_uart) {

		case 'a':
					brillo_led_aumentar(dato_uart, duty_cycle);
					break;
		case 'd':
					brillo_led_disminuir(dato_uart, duty_cycle);
					break;
		default:
					stdioPrintf(UART_USB, "Ha ingresado un caracter invalido. \n \r");
					break;
	}
}

void brillo_led_aumentar(uint8_t dato_uart, uint8_t duty_cycle) {

	switch(dato_uart) {

		case '1':
					if( duty_cycle[0] > BRILLO_MAXIMO)	stdioPrintf(UART_USB, "LED 1 brillo maximo! \n \r");
					else {
							duty_cycle[0] += 25;
					}

					break;

		case '2':
					if( duty_cycle[1] > BRILLO_MAXIMO)	stdioPrintf(UART_USB, "LED 2 brillo maximo! \n \r");
					else {
							duty_cycle[1] += 25;
					}

					break;

		case '3':
					if( duty_cycle[2] > BRILLO_MAXIMO)	stdioPrintf(UART_USB, "LED 3 brillo maximo! \n \r");
					else {
							duty_cycle[2] += 25;
					}

					break;

		default:
					printf("Ha ingresado un valor invalido. \n \r");
					break;

	}
}

void brillo_led_disminuir(uint8_t dato_uart, uint8_t duty_cycle) {

	switch(dato_uart) {

		case '1':
					if( duty_cycle[0] < BRILLO_MINIMO)	stdioPrintf(UART_USB, "LED 1 brillo maximo! \n \r");
					else {
							duty_cycle[0] -= 25;
					}

					break;

		case '2':
					if( duty_cycle[1] < BRILLO_MINIMO)	stdioPrintf(UART_USB, "LED 2 brillo maximo! \n \r");
					else {
							duty_cycle[1] -= 25;
					}

					break;

		case '3':
					if( duty_cycle[2] < BRILLO_MINIMO)	stdioPrintf(UART_USB, "LED 3 brillo maximo! \n \r");
					else {
							duty_cycle[2] -= 25;
					}

					break;

		default:
					printf("Ha ingresado un valor invalido. \n \r");
					break;
	}
}
