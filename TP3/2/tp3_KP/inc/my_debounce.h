/*
 * my_debounce.h
 */

#ifndef EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_MY_DEBOUNCE_H_
#define EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_MY_DEBOUNCE_H_


#include "sapi.h"
#include "my_gpio.h"

// De examples/c/sapi/finite_state_machine/fsm_debounce/
typedef enum{
   STATE_BUTTON_UP,
   STATE_BUTTON_DOWN,
   STATE_BUTTON_FALLING,
   STATE_BUTTON_RISING
} my_ButtonState_t;

my_ButtonState_t my_ButtonError( void );
my_ButtonState_t my_ButtonInit( void );
void my_ButtonUpdate( my_gpio_map_t tecla, bool_t *transicion );

#endif /* EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_MY_DEBOUNCE_H_ */
