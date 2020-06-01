#include "build/temp/_test_my_gpio.c"
#include "src/my_gpio.h"
#include "build/test/mocks/mock_gpio_18xx_43xx.h"
#include "build/test/mocks/mock_scu_18xx_43xx.h"
#include "/home/matias/.rbenv/versions/2.7.1/lib/ruby/gems/2.7.0/gems/ceedling-0.30.0/vendor/unity/src/unity.h"






void setUp(void) {



}



void tearDown(void) {

}



void test_my_gpio_init_input(void) {





 Chip_SCU_PinMux_CMockExpect(

       19

 , 1, 1, 0, 0)

        ;

 Chip_GPIO_SetDir_CMockExpect(

                    21

 , ((LPC_GPIO_T *) 0x400F4000), 1, ( 1 << 1 ), 1)

                     ;



 my_gpio_init(LED1, MY_GPIO_INPUT);



}
