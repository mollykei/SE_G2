#include "my_gpio.h"

#include "scu_18xx_43xx.h"
#include "gpio_18xx_43xx.h"

void my_gpio_init(int8_t pinNamePort, int8_t pinNamePin, int8_t func) {


	Chip_SCU_PinMux(
         pinNamePort,
         pinNamePin,
         SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
         func
      );


	return ;
}
