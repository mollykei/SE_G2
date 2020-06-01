#include "my_gpio.h"

#include "scu_18xx_43xx.h"
#include "gpio_18xx_43xx.h"

// Private struct
struct _my_gpio_pins_t {
	uint8_t scu_pin;
	uint8_t scu_port;
	uint8_t gpio_pin;
	uint8_t gpio_port;
	uint8_t func;
};
// struct alias
typedef struct _my_gpio_pins_t _my_gpio_pins_t;

// Private instance
//TODO: fill in
const _my_gpio_pins_t gpio_pins_init[] =
		{ { 1, 1, 1, 1, 0 }, { 2, 3, 4, 5, 6 } };
// Private functions
static _my_gpio_pins_t gpioGetPin(my_gpio_map_t pin) {
	return (gpio_pins_init[pin]);
}

// Public functions
void my_gpio_init(my_gpio_map_t pin, my_gpio_config_t config) {

	const _my_gpio_pins_t pins = gpioGetPin(pin);

	switch (config) {

	case MY_GPIO_INPUT:
		Chip_SCU_PinMux(pins.scu_pin, pins.scu_port, 0, pins.func);
		Chip_GPIO_SetDir(LPC_GPIO_PORT, pins.gpio_pin, (1 << pins.gpio_port),
				config);
		break;

	default:
		;

	}

	return;
}
