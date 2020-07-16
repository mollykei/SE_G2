/*
 * my_debounce.h
 */

#ifndef EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_MY_DEBOUNCE_H_
#define EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_MY_DEBOUNCE_H_

#ifdef TEST_CEEDLING
//cp_mcu_scu_gpio.h is a header file with out dependencies, just to mock the functions
#include "cp_mcu_scu_gpio.h"
#else
#include "sapi.h"
#include "my_gpio.h"
#endif


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

#ifdef __cplusplus
}
#endif

#endif /* EXAMPLES_C_PROJECTS_TP3_PUNTO7_BUTTON2LED_INC_MY_DEBOUNCE_H_ */
