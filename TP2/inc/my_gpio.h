#ifndef MY_GPIO_H
#define MY_GPIO_H

#include <stdbool.h>

#ifdef TEST_CEEDLING
//cp_mcu_scu_gpio.h is a header file with out dependencies, just to mock the functions
#include "cp_mcu_scu_gpio.h"
#else
#include "sapi.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// edu-ciaa pins
typedef enum {
	MY_GPIO_LED1,
	MY_GPIO_LED2,
	MY_GPIO_LED3,
	MY_GPIO_LEDR,
	MY_GPIO_LEDG,
	MY_GPIO_LEDB,
	MY_GPIO_TEC1,
	MY_GPIO_TEC2,
	MY_GPIO_TEC3,
	MY_GPIO_TEC4,
	MY_GPIO_GPIO0,
	MY_GPIO_GPIO1,
} my_gpio_map_t;

//Max gpio map pines
#define MAX_GPIO_PINS 11

// configs
typedef enum {
	MY_GPIO_INPUT,
	MY_GPIO_OUTPUT,
	MY_GPIO_INPUT_PULLDOWN,
	MY_GPIO_INPUT_PULLUP,
	MY_GPIO_INPUT_PULLUP_PULLDOWN,
} my_gpio_config_t;

//Tell the compiler that the private struct exist
//but I don't tell what it has inside
// I can use this outside the module
struct _my_gpio_pins_t;
typedef struct _my_gpio_pins_t * my_gpio_pins_t;

/*
 * @brief init gpio port
 * @param pin mcu pin that need init
 * @param config ping configuration
 * @return nothing
 * @extra SCU MODES: from datasheet page 395
 * 	- Enable input buffer in order to read digital signals
 * 	- The glitch filter is enable by default, disable it
 * 	- The on-chip pull-up or pull-down resistor have a tipical value of 50k
 * 	- The default value is pull-up enable
 */
void my_gpio_init(my_gpio_map_t const pin, my_gpio_config_t const config);

/*
 * @brief gprio write value
 * @param pin mcu pin to be write
 * @param val bool value
 * @return nothing
 */
void my_gpio_write(my_gpio_map_t const pin, bool const val);

/*
 * @brief gpio read pin
 * @param pin mcu pin to be read
 * @return readed value
 */
bool my_gpio_read(my_gpio_map_t const pin);

#ifdef __cplusplus
}
#endif


#endif // MY_GPIO_H
