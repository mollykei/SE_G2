#include "unity.h"

#include "mock_scu_18xx_43xx.h"
#include "mock_gpio_18xx_43xx.h"

#include "my_gpio.h"



void setUp(void) {
}

void tearDown(void) {
}

void test_my_gpio_NeedToImplement(void) {

	Chip_SCU_PinMux_Expect(1, 1,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 1);

	my_gpio_init(1, 2, 1);

}
