#include "unity.h"
#include "mock_cp_mcu_scu_gpio.h"
#include "my_gpio.h"

void setUp(void) {

}

void tearDown(void) {
}

void test_my_gpio_write(void) {

	Chip_GPIO_SetPinState_Expect(LPC_GPIO_PORT, 5, 0, 0);
	my_gpio_write(MY_GPIO_LEDR, 0);

	Chip_GPIO_SetPinState_Expect(LPC_GPIO_PORT, 5, 0, 1);
	my_gpio_write(MY_GPIO_LEDR, 1);
}

void test_my_gpio_read_outOfRange(void) {

	TEST_ASSERT_FALSE(my_gpio_read(12));
	TEST_ASSERT_FALSE(my_gpio_read(-123));
	TEST_ASSERT_FALSE(my_gpio_read(-1129312));
}

void test_my_gpio_read(void) {

	Chip_GPIO_ReadPortBit_ExpectAndReturn(LPC_GPIO_PORT, 0, 14, 1);
	TEST_ASSERT_TRUE(my_gpio_read(MY_GPIO_LED1));

	Chip_GPIO_ReadPortBit_ExpectAndReturn(LPC_GPIO_PORT, 0, 14, 0);
	TEST_ASSERT_FALSE(my_gpio_read(MY_GPIO_LED1));

}

void test_my_gpio_init_output(void) {

	Chip_SCU_PinMux_Expect(2, 10,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 0, (1 << 14), MY_GPIO_OUTPUT);
	Chip_GPIO_SetPinState_Expect(LPC_GPIO_PORT, 0, 14, 0);
	my_gpio_init(MY_GPIO_LED1, MY_GPIO_OUTPUT);

}

void test_my_gpio_init_input_pullup_pulldown(void) {

	Chip_SCU_PinMux_Expect(2, 10,
			SCU_MODE_REPEATER | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 0, (1 << 14), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_LED1, MY_GPIO_INPUT_PULLUP_PULLDOWN);

}

void test_my_gpio_init_input_pullup(void) {

	Chip_SCU_PinMux_Expect(2, 10,
			SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 0, (1 << 14), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_LED1, MY_GPIO_INPUT_PULLUP);

}

void test_my_gpio_init_input_pulldown(void) {

	Chip_SCU_PinMux_Expect(2, 10,
			SCU_MODE_PULLDOWN | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 0, (1 << 14), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_LED1, MY_GPIO_INPUT_PULLDOWN);

	Chip_SCU_PinMux_Expect(6, 1,
			SCU_MODE_PULLDOWN | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 3, (1 << 0), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_GPIO0, MY_GPIO_INPUT_PULLDOWN);

}

void test_my_gpio_init_input(void) {

	Chip_SCU_PinMux_Expect(2, 10,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 0, (1 << 14), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_LED1, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(2, 11,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 1, (1 << 11), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_LED2, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(2, 12,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 1, (1 << 12), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_LED3, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(2, 0,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 4);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 5, (1 << 0), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_LEDR, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(2, 1,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 4);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 5, (1 << 1), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_LEDG, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(2, 2,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 4);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 5, (1 << 2), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_LEDB, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(1, 0,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 0, (1 << 4), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_TEC1, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(1, 1,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 0, (1 << 8), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_TEC2, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(1, 2,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 0, (1 << 9), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_TEC3, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(1, 6,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 1, (1 << 9), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_TEC4, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(6, 1,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 3, (1 << 0), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_GPIO0, MY_GPIO_INPUT);

	Chip_SCU_PinMux_Expect(6, 4,
			SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS, 0);
	Chip_GPIO_SetDir_Expect(LPC_GPIO_PORT, 3, (1 << 3), MY_GPIO_INPUT);
	my_gpio_init(MY_GPIO_GPIO1, MY_GPIO_INPUT);

}
