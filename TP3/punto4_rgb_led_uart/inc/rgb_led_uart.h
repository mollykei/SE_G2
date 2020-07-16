/*
 * rgb_led_uart.h
 */

#ifndef EXAMPLES_C_PROJECTS_TP3_PUNTO4_RGB_LED_UART_RGB_LED_UART_H_
#define EXAMPLES_C_PROJECTS_TP3_PUNTO4_RGB_LED_UART_RGB_LED_UART_H_

// Bibliotecas SAPI
#include "sapi.h"
#include "sapi_pwm.h"
#include "sapi_uart.h"
#include "sapi_board.h"


#define BRILLO_MAXIMO 255
#define BRILLO_MINIMO 0

// Prototipos
void inicializar_plataforma();
void pwm_configuracion();
void menu_mensajes_brillo();
void menu_mensajes_led();

void brillo_seleccion_modo(uint8_t dato_uart, uint8_t duty_cycle);
void brillo_led_aumentar(uint8_t dato_uart, uint8_t duty_cycle);
void brillo_led_disminuir(uint8_t dato_uart, uint8_t duty_cycle);

#endif /* EXAMPLES_C_PROJECTS_TP3_PUNTO4_RGB_LED_UART_RGB_LED_UART_H_ */

