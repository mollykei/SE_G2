#include "my_gpio.h"

// Private struct
struct _my_gpio_pins_t {
	uint8_t scu_port;
	uint8_t scu_pin;
	uint8_t gpio_port;
	uint8_t gpio_pin;
	uint8_t func;
};
// struct alias
typedef struct _my_gpio_pins_t _my_gpio_pins_t;

// Private instance of _my_gpio_pins_t
//TODO: fill in
const _my_gpio_pins_t gpio_pins_init[] = {
		//{scu_port, scu_pin, gpio_port, gpio_pin, function}
		{ 2, 10, 0, 14, 0 }, //LED1
		{ 2, 11, 1, 11, 0 }, //LED2
		{ 2, 12, 1, 12, 0 }, //LED3
		{ 2, 0, 5, 0, 4 },   //LEDR
		{ 2, 1, 5, 1, 4 },   //LEDG
		{ 2, 2, 5, 2, 4 },   //LEDB
		{ 1, 0, 0, 4, 0 },   //TEC1
		{ 1, 1, 0, 8, 0 },   //TEC2
		{ 1, 2, 0, 9, 0 },   //TEC3
		{ 1, 6, 1, 9, 0 },   //TEC4
		{ 6, 1, 3, 0, 0 },   //GPIO0
		{ 6, 4, 3, 3, 0 },   //GPIO1
};

// Private functions
static my_gpio_pins_t gpioGetPin(my_gpio_map_t const pin) {
	return (my_gpio_pins_t) &(gpio_pins_init[pin]);
}

static bool checkRange(my_gpio_map_t const pin) {
	return pin < 0 || pin > MAX_GPIO_PINS;
}

static uint32_t pinToBit(uint8_t pin) {
	return (1 << pin);
}

// Public functions
void my_gpio_init(my_gpio_map_t const pin, my_gpio_config_t const config) {

	if (checkRange(pin)) {
		return;
	}

	my_gpio_pins_t mcu_pin = gpioGetPin(pin);

	switch (config) {

	case MY_GPIO_INPUT:
		Chip_SCU_PinMux(mcu_pin->scu_port, mcu_pin->scu_pin,
				SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
				mcu_pin->func);
		Chip_GPIO_SetDir(LPC_GPIO_PORT, mcu_pin->gpio_port,
				pinToBit(mcu_pin->gpio_pin), MY_GPIO_INPUT);
		break;

	case MY_GPIO_INPUT_PULLUP:
		Chip_SCU_PinMux(mcu_pin->scu_port, mcu_pin->scu_pin,
				SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
				mcu_pin->func);
		Chip_GPIO_SetDir(LPC_GPIO_PORT, mcu_pin->gpio_port,
				pinToBit(mcu_pin->gpio_pin), MY_GPIO_INPUT);
		break;

	case MY_GPIO_INPUT_PULLDOWN:
		Chip_SCU_PinMux(mcu_pin->scu_port, mcu_pin->scu_pin,
				SCU_MODE_PULLDOWN | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
				mcu_pin->func);
		Chip_GPIO_SetDir(LPC_GPIO_PORT, mcu_pin->gpio_port,
				pinToBit(mcu_pin->gpio_pin), MY_GPIO_INPUT);
		break;

	case MY_GPIO_INPUT_PULLUP_PULLDOWN:
		Chip_SCU_PinMux(mcu_pin->scu_port, mcu_pin->scu_pin,
				SCU_MODE_REPEATER | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
				mcu_pin->func);
		Chip_GPIO_SetDir(LPC_GPIO_PORT, mcu_pin->gpio_port,
				pinToBit(mcu_pin->gpio_pin), MY_GPIO_INPUT);
		break;
	case MY_GPIO_OUTPUT:
		Chip_SCU_PinMux(mcu_pin->scu_port, mcu_pin->scu_pin,
				SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
				mcu_pin->func);
		Chip_GPIO_SetDir(LPC_GPIO_PORT, mcu_pin->gpio_port,
				pinToBit(mcu_pin->gpio_pin), MY_GPIO_OUTPUT);
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, mcu_pin->gpio_port,
				mcu_pin->gpio_pin, 0);
		break;

	default:
		//TODO check for errors?
		;

	}

	return;
}

bool my_gpio_read(my_gpio_map_t const pin) {

	if (checkRange(pin)) {
		return false;
	}

	bool tmp_val;

	my_gpio_pins_t mcu_pin = gpioGetPin(pin);
	tmp_val = Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, mcu_pin->gpio_port,
			mcu_pin->gpio_pin);

	return tmp_val;

}

void my_gpio_write(my_gpio_map_t const pin, bool const val) {

	if (checkRange(pin)) {
		return;
	}
	my_gpio_pins_t mcu_pin = gpioGetPin(pin);

	Chip_GPIO_SetPinState(LPC_GPIO_PORT, mcu_pin->gpio_port, mcu_pin->gpio_pin,
			val);

	return;
}

