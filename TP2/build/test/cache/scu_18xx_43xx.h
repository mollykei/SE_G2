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

static inline void Chip_SCU_PinMuxSet(uint8_t port, uint8_t pin, uint16_t modefunc)

{

 LPC_SCU->SFSP[port][pin] = modefunc;

}

static inline void Chip_SCU_PinMux(uint8_t port, uint8_t pin, uint16_t mode, uint8_t func)

{

 Chip_SCU_PinMuxSet(port, pin, (mode | (uint16_t) func));

}















static inline void Chip_SCU_ClockPinMuxSet(uint8_t clknum, uint16_t modefunc)

{

 LPC_SCU->SFSCLK[clknum] = (uint32_t) modefunc;

}

static inline void Chip_SCU_ClockPinMux(uint8_t clknum, uint16_t mode, uint8_t func)

{

 LPC_SCU->SFSCLK[clknum] = ((uint32_t) mode | (uint32_t) func);

}

static inline void Chip_SCU_GPIOIntPinSel(uint8_t PortSel, uint8_t PortNum, uint8_t PinNum)

{

 int32_t of = (PortSel & 3) << 3;

 uint32_t val = (((PortNum & 0x7) << 5) | (PinNum & 0x1F)) << of;

 LPC_SCU->PINTSEL[PortSel >> 2] = (LPC_SCU->PINTSEL[PortSel >> 2] & ~(0xFF << of)) | val;

}

static inline void Chip_SCU_I2C0PinConfig(uint32_t I2C0Mode)

{

 LPC_SCU->SFSI2C0 = I2C0Mode;

}















static inline void Chip_SCU_ADC_Channel_Config(uint32_t ADC_ID, uint8_t channel)

{

 LPC_SCU->ENAIO[ADC_ID] |= 1UL << channel;

}











static inline void Chip_SCU_DAC_Analog_Config(void)

{



 LPC_SCU->ENAIO[2] |= 1;

}















static inline void Chip_SCU_SetPinMuxing(const PINMUX_GRP_T *pinArray, uint32_t arrayLength)

{

 uint32_t ix;

 for (ix = 0; ix < arrayLength; ix++ ) {

  Chip_SCU_PinMuxSet(pinArray[ix].pingrp, pinArray[ix].pinnum, pinArray[ix].modefunc);

 }

}
