/*
 * button2led.h
 *
 *  Created on: Jul 16, 2020
 *      Author: nandroid
 */

#ifndef EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_BUTTON2LED_H_
#define EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_BUTTON2LED_H_

#ifdef TEST_CEEDLING
//cp_mcu_scu_gpio.h is a header file with out dependencies, just to mock the functions
#include "cp_mcu_scu_gpio.h"
#else
#include "sapi.h"
#include "my_gpio.h"
#include "my_debounce.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


void inicializar_plataforma(void);
void my_PrintChar(gpioMap_t tecla, my_ButtonState_t buttonState);




#endif /* EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_BUTTON2LED_H_ */
