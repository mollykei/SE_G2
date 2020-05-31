typedef struct {

 uint8_t pingrp;

 uint8_t pinnum;

 uint16_t modefunc;

} PINMUX_GRP_T;









typedef struct {

 volatile uint32_t SFSP[16][32];

 volatile uint32_t RESERVED0[256];

 volatile uint32_t SFSCLK[4];

 volatile uint32_t RESERVED16[28];

 volatile uint32_t SFSUSB;

 volatile uint32_t SFSI2C0;

 volatile uint32_t ENAIO[3];

 volatile uint32_t RESERVED17[27];

 volatile uint32_t EMCDELAYCLK;

 volatile uint32_t RESERVED18[63];

 volatile uint32_t PINTSEL[2];

} LPC_SCU_T;

void Chip_SCU_PinMuxSet(uint8_t port, uint8_t pin, uint16_t modefunc);

void Chip_SCU_PinMux(uint8_t port, uint8_t pin, uint16_t mode, uint8_t func);















void Chip_SCU_ClockPinMuxSet(uint8_t clknum, uint16_t modefunc);

void Chip_SCU_ClockPinMux(uint8_t clknum, uint16_t mode, uint8_t func);

void Chip_SCU_GPIOIntPinSel(uint8_t PortSel, uint8_t PortNum, uint8_t PinNum);

void Chip_SCU_I2C0PinConfig(uint32_t I2C0Mode);















void Chip_SCU_ADC_Channel_Config(uint32_t ADC_ID, uint8_t channel);











void Chip_SCU_DAC_Analog_Config(void);















void Chip_SCU_SetPinMuxing(const PINMUX_GRP_T *pinArray, uint32_t arrayLength);
