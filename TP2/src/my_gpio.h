#ifndef MY_GPIO_H
#define MY_GPIO_H

#include <stdint.h>
#include <stdbool.h>

//Esto lo tengo que ponger xq cmock es una verga
#define SCU_MODE_PULLUP            (0x0 << 3)		/*!< Enable pull-up resistor at pad */
#define SCU_MODE_REPEATER          (0x1 << 3)		/*!< Enable pull-down and pull-up resistor at resistor at pad (repeater mode) */
#define SCU_MODE_INACT             (0x2 << 3)		/*!< Disable pull-down and pull-up resistor at resistor at pad */
#define SCU_MODE_PULLDOWN          (0x3 << 3)		/*!< Enable pull-down resistor at pad */
#define SCU_MODE_HIGHSPEEDSLEW_EN  (0x1 << 5)		/*!< Enable high-speed slew */
#define SCU_MODE_INBUFF_EN         (0x1 << 6)		/*!< Enable Input buffer */
#define SCU_MODE_ZIF_DIS           (0x1 << 7)		/*!< Disable input glitch filter */
#define SCU_MODE_4MA_DRIVESTR      (0x0 << 8)		/*!< Normal drive: 4mA drive strength */
#define SCU_MODE_8MA_DRIVESTR      (0x1 << 8)		/*!< Medium drive: 8mA drive strength */
#define SCU_MODE_14MA_DRIVESTR     (0x2 << 8)		/*!< High drive: 14mA drive strength */
#define SCU_MODE_20MA_DRIVESTR     (0x3 << 8)		/*!< Ultra high- drive: 20mA drive strength */
#define SCU_MODE_FUNC0             0x0				/*!< Selects pin function 0 */
#define SCU_MODE_FUNC1             0x1				/*!< Selects pin function 1 */
#define SCU_MODE_FUNC2             0x2				/*!< Selects pin function 2 */
#define SCU_MODE_FUNC3             0x3				/*!< Selects pin function 3 */
#define SCU_MODE_FUNC4             0x4				/*!< Selects pin function 4 */
#define SCU_MODE_FUNC5             0x5				/*!< Selects pin function 5 */
#define SCU_MODE_FUNC6             0x6				/*!< Selects pin function 6 */
#define SCU_MODE_FUNC7             0x7				/*!< Selects pin function 7 */
#define SCU_PINIO_FAST             (SCU_MODE_INACT | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS)


void my_gpio_init(int8_t pinNamePort, int8_t pinNamePin, int8_t func);

#endif // MY_GPIO_H
