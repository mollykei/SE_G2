/*
 * button2led.h
 *
 *  Created on: Jul 16, 2020
 *      Author: nandroid
 */

#ifndef EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_BUTTON2LED_H_
#define EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_BUTTON2LED_H_

#include "sapi.h"
#include "my_gpio.h"
#include "my_debounce.h"

void inicializar_plataforma();
void onRx(void* noUsado);
void my_PrintChar(gpioMap_t tecla, my_ButtonState_t buttonState);


#endif /* EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_BUTTON2LED_H_ */
