#include "unity.h"

#include "mock_scu_18xx_43xx.h"
#include "mock_gpio_18xx_43xx.h"

#include "my_gpio.h"

void setUp(void) {

}

void tearDown(void) {
}

void test_my_gpio_init_input(void) {

	//Expected for gpio_init_input
	Chip_SCU_PinMux_Expect(1, 1,
			0, 0);
	Chip_GPIO_SetDir_Expect( LPC_GPIO_PORT,
			1, ( 1 << 1 ), 1 );

	my_gpio_init(LED1, MY_GPIO_INPUT);

}
