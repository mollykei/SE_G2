#ifndef EXAMPLES_C_TP2_TP2_SRC_TP2_C_
#define EXAMPLES_C_TP2_TP2_SRC_TP2_C_

#include <stdbool.h>
#include "my_gpio.h"

int main(void) {

	SystemCoreClockUpdate();
	cyclesCounterInit(SystemCoreClock);

#ifndef USE_FREERTOS
	tickInit(1);
#endif

	my_gpio_init(MY_GPIO_LEDR, MY_GPIO_OUTPUT);

	while (1) {

		my_gpio_write(MY_GPIO_LEDR, ON);
		delay(1000);
		my_gpio_write(MY_GPIO_LEDR, OFF);
		delay(1000);

	}

	return 0;
}

#endif /* EXAMPLES_C_TP2_TP2_SRC_TP2_C_ */
