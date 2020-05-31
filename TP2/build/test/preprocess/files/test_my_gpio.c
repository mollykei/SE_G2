#include "build/temp/_test_my_gpio.c"
#include "src/my_gpio.h"
#include "mock_gpio_18xx_43xx.h"
#include "mock_scu_18xx_43xx.h"
#include "/home/matias/.rbenv/versions/2.7.1/lib/ruby/gems/2.7.0/gems/ceedling-0.30.0/vendor/unity/src/unity.h"










void setUp(void) {

}



void tearDown(void) {

}



void test_my_gpio_NeedToImplement(void) {



 Chip_SCU_PinMux_CMockExpect(

                                                            19

 , 1, 1, (0x2 << 3) | (0x1 << 6) | (0x1 << 7), 1)

                                                             ;



 my_gpio_init(1, 2, 1);



}
