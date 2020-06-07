#ifndef _CP_MCU_SCU_GPIO_H_
#define _CP_MCU_SCU_GPIO_H_

#ifdef TEST_CEEDLING
#include <stdint.h>
#include <stdbool.h>

#define __IO 	volatile
#define __O 	volatile
#define __I 	volatile


typedef struct {
	uint8_t pingrp;		/* Pin group */
	uint8_t pinnum;		/* Pin number */
	uint16_t modefunc;	/* Pin mode and function for SCU */
} PINMUX_GRP_T;


typedef struct {
	__IO uint32_t  SFSP[16][32];
	__I  uint32_t  RESERVED0[256];
	__IO uint32_t  SFSCLK[4];			/*!< Pin configuration register for pins CLK0-3 */
	__I  uint32_t  RESERVED16[28];
	__IO uint32_t  SFSUSB;				/*!< Pin configuration register for USB */
	__IO uint32_t  SFSI2C0;				/*!< Pin configuration register for I2C0-bus pins */
	__IO uint32_t  ENAIO[3];			/*!< Analog function select registerS */
	__I  uint32_t  RESERVED17[27];
	__IO uint32_t  EMCDELAYCLK;			/*!< EMC clock delay register */
	__I  uint32_t  RESERVED18[63];
	__IO uint32_t  PINTSEL[2];			/*!< Pin interrupt select register for pin int 0 to 3 index 0, 4 to 7 index 1. */
} LPC_SCU_T;

typedef struct {				/*!< GPIO_PORT Structure */
	__IO uint8_t B[128][32];	/*!< Offset 0x0000: Byte pin registers ports 0 to n; pins PIOn_0 to PIOn_31 */
	__IO uint32_t W[32][32];	/*!< Offset 0x1000: Word pin registers port 0 to n */
	__IO uint32_t DIR[32];		/*!< Offset 0x2000: Direction registers port n */
	__IO uint32_t MASK[32];		/*!< Offset 0x2080: Mask register port n */
	__IO uint32_t PIN[32];		/*!< Offset 0x2100: Portpin register port n */
	__IO uint32_t MPIN[32];		/*!< Offset 0x2180: Masked port register port n */
	__IO uint32_t SET[32];		/*!< Offset 0x2200: Write: Set register for port n Read: output bits for port n */
	__O  uint32_t CLR[32];		/*!< Offset 0x2280: Clear port n */
	__O  uint32_t NOT[32];		/*!< Offset 0x2300: Toggle port n */
} LPC_GPIO_T;


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

#define LPC_GPIO_PORT_BASE        0x400F4000
#define LPC_GPIO_PORT             ((LPC_GPIO_T             *) LPC_GPIO_PORT_BASE)


void Chip_SCU_PinMuxSet(uint8_t port, uint8_t pin, uint16_t modefunc);
void Chip_SCU_PinMux(uint8_t port, uint8_t pin, uint16_t mode, uint8_t func);
void Chip_GPIO_SetPinState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, bool setting);
void Chip_GPIO_SetDir(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue, uint8_t out);
bool Chip_GPIO_ReadPortBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin);


#endif
#endif
