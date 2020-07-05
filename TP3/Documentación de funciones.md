## TP3 - Documentación de funciones


 1. uartConfig( UART_USB, 115200 );


En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_uart.c` se encuentra la definición de `uartConfig`

```C
#define uartConfig uartInit
```

```C
// UART Initialization
void uartInit( uartMap_t uart, uint32_t baudRate )
{
   // Initialize UART
   Chip_UART_Init( lpcUarts[uart].uartAddr );
   // Set Baud rate
   Chip_UART_SetBaud( lpcUarts[uart].uartAddr, baudRate );

   //Chip_UART_ConfigData(LPC_UART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));

   // Restart FIFOS using FCR (FIFO Control Register).
   // Set Enable, Reset content, set trigger level
   Chip_UART_SetupFIFOS( lpcUarts[uart].uartAddr,
                         UART_FCR_FIFO_EN |
                         UART_FCR_TX_RS   |
                         UART_FCR_RX_RS   |
                         UART_FCR_TRG_LEV0 );
	/*Chip_UART_SetupFIFOS(lpcUarts[uart].uartAddr,
                          (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));*/

   // Dummy read
   Chip_UART_ReadByte( lpcUarts[uart].uartAddr );

   // Enable UART Transmission
   Chip_UART_TXEnable( lpcUarts[uart].uartAddr );

   // Configure SCU UARTn_TXD pin
   Chip_SCU_PinMux( lpcUarts[uart].txPin.lpcScuPort,
                    lpcUarts[uart].txPin.lpcScuPin,
                    MD_PDN,
                    lpcUarts[uart].txPin.lpcScuFunc );

   // Configure SCU UARTn_RXD pin
   Chip_SCU_PinMux( lpcUarts[uart].rxPin.lpcScuPort,
                    lpcUarts[uart].rxPin.lpcScuPin,
                    MD_PLN | MD_EZI | MD_ZI,
                    lpcUarts[uart].rxPin.lpcScuFunc );

   // Specific configurations for RS485
   if( uart == UART_485 ) {
      // Specific RS485 Flags
      Chip_UART_SetRS485Flags( LPC_USART0,
                               UART_RS485CTRL_DCTRL_EN |
                               UART_RS485CTRL_OINV_1     );
      // UARTn_DIR extra pin for RS485
      Chip_SCU_PinMux( lpcUart485DirPin.lpcScuPort,
                       lpcUart485DirPin.lpcScuPin,
                       MD_PDN,
                       lpcUart485DirPin.lpcScuFunc );
   }
}
```

En `firmware_v3/libs/sapi/sapi_v0.5.2/board/inc/sapi_peripheral_map.h` se define la estructura `uartMap_t`, que según el tipo de placa, para nuestro caso la ***edu_ciaa_nxp*** se define que ` UART_USB =  3`
```C
typedef enum {
	#if (BOARD == ciaa_nxp)
	   UART_485  = 1, // Hardware UART0 via RS_485 A, B and GND Borns
					  // Hardware UART1 not routed
	   UART_USB  = 3, // Hardware UART2 via USB DEBUG port
	   UART_232  = 5, // Hardware UART3 via 232_RX and 232_tx pins on header P1
	#elif (BOARD == edu_ciaa_nxp)
	   UART_GPIO = 0, // Hardware UART0 via GPIO1(TX), GPIO2(RX) pins on header P0
	   UART_485  = 1, // Hardware UART0 via RS_485 A, B and GND Borns
		// Hardware UART1 not routed
	   UART_USB  = 3, // Hardware UART2 via USB DEBUG port
	   UART_ENET = 4, // Hardware UART2 via ENET_RXD0(TX), ENET_CRS_DV(RX) pins on header P0
	   UART_232  = 5, // Hardware UART3 via 232_RX and 232_tx pins on header P1
	#else
	   #error BOARD not supported yet!
	#endif
   UART_MAXNUM,
} uartMap_t;
```

Volviendo a la primitiva de `uartInit`, se ve que en primer lugar inicializa el UART, en `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/uart_18xx_43xx.c`:

```C
/* Initializes the pUART peripheral */
void Chip_UART_Init(LPC_USART_T *pUART)
{
    volatile uint32_t tmp;

	/* Enable UART clocking. UART base clock(s) must already be enabled */
	Chip_Clock_EnableOpts(UART_PClock[Chip_UART_GetIndex(pUART)], true, true, 1);

	/* Enable FIFOs by default, reset them */
	Chip_UART_SetupFIFOS(pUART, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS));

    /* Disable Tx */
    Chip_UART_TXDisable(pUART);

    /* Disable interrupts */
	pUART->IER = 0;
	/* Set LCR to default state */
	pUART->LCR = 0;
	/* Set ACR to default state */
	pUART->ACR = 0;
    /* Set RS485 control to default state */
	pUART->RS485CTRL = 0;
	/* Set RS485 delay timer to default state */
	pUART->RS485DLY = 0;
	/* Set RS485 addr match to default state */
	pUART->RS485ADRMATCH = 0;

    /* Clear MCR */
    if (pUART == LPC_UART1) {
		/* Set Modem Control to default state */
		pUART->MCR = 0;
		/*Dummy Reading to Clear Status */
		tmp = pUART->MSR;
	}

	/* Default 8N1, with DLAB disabled */
	Chip_UART_ConfigData(pUART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS));

	/* Disable fractional divider */
	pUART->FDR = 0x10;

    (void) tmp;
}
```

Y luego setea el baudrate


```C
/* Determines and sets best dividers to get a target bit rate */
uint32_t Chip_UART_SetBaud(LPC_USART_T *pUART, uint32_t baudrate)
{
	uint32_t div, divh, divl, clkin;

	/* Determine UART clock in rate without FDR */
	clkin = Chip_Clock_GetRate(UART_BClock[Chip_UART_GetIndex(pUART)]);
	div = clkin / (baudrate * 16);

	/* High and low halves of the divider */
	divh = div / 256;
	divl = div - (divh * 256);

	Chip_UART_EnableDivisorAccess(pUART);
	Chip_UART_SetDivisorLatches(pUART, divl, divh);
	Chip_UART_DisableDivisorAccess(pUART);

	/* Fractional FDR alreadt setup for 1 in UART init */

	return (clkin / div) >> 4;
}
```

También se habilita la trasnmisión en el pin Tx en `/firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/uart_18xx_43xx.h` y se describen los registros que se utilizan, como `TER2`, `ÌER`:

```C
/**
 * @brief	Enable transmission on UART TxD pin
 * @param	pUART	: Pointer to selected pUART peripheral
 * @return Nothing
 */
STATIC INLINE void Chip_UART_TXEnable(LPC_USART_T *pUART)
{
    pUART->TER2 = UART_TER2_TXEN;
}
```

```C
#define UART_TER2_TXEN      (1 << 0)		/*!< Transmit enable bit  - valid for 18xx/43xx only */
```

```C
/**
 * @brief USART register block structure
 */
typedef struct {					/*!< USARTn Structure       */

	union {
		__IO uint32_t  DLL;			/*!< Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider (DLAB = 1). */
		__O  uint32_t  THR;			/*!< Transmit Holding Register. The next character to be transmitted is written here (DLAB = 0). */
		__I  uint32_t  RBR;			/*!< Receiver Buffer Register. Contains the next received character to be read (DLAB = 0). */
	};

	union {
		__IO uint32_t IER;			/*!< Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential UART interrupts (DLAB = 0). */
		__IO uint32_t DLM;			/*!< Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider (DLAB = 1). */
	};

	union {
		__O  uint32_t FCR;			/*!< FIFO Control Register. Controls UART FIFO usage and modes. */
		__I  uint32_t IIR;			/*!< Interrupt ID Register. Identifies which interrupt(s) are pending. */
	};

	__IO uint32_t LCR;				/*!< Line Control Register. Contains controls for frame formatting and break generation. */
	__IO uint32_t MCR;				/*!< Modem Control Register. Only present on USART ports with full modem support. */
	__I  uint32_t LSR;				/*!< Line Status Register. Contains flags for transmit and receive status, including line errors. */
	__I  uint32_t MSR;				/*!< Modem Status Register. Only present on USART ports with full modem support. */
	__IO uint32_t SCR;				/*!< Scratch Pad Register. Eight-bit temporary storage for software. */
	__IO uint32_t ACR;				/*!< Auto-baud Control Register. Contains controls for the auto-baud feature. */
	__IO uint32_t ICR;				/*!< IrDA control register (not all UARTS) */
	__IO uint32_t FDR;				/*!< Fractional Divider Register. Generates a clock input for the baud rate divider. */
	__IO uint32_t OSR;				/*!< Oversampling Register. Controls the degree of oversampling during each bit time. Only on some UARTS. */
	__IO uint32_t TER1;				/*!< Transmit Enable Register. Turns off USART transmitter for use with software flow control. */
	uint32_t  RESERVED0[3];
    __IO uint32_t HDEN;				/*!< Half-duplex enable Register- only on some UARTs */
	__I  uint32_t RESERVED1[1];
	__IO uint32_t SCICTRL;			/*!< Smart card interface control register- only on some UARTs */

	__IO uint32_t RS485CTRL;		/*!< RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes. */
	__IO uint32_t RS485ADRMATCH;	/*!< RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode. */
	__IO uint32_t RS485DLY;			/*!< RS-485/EIA-485 direction control delay. */

	union {
		__IO uint32_t SYNCCTRL;		/*!< Synchronous mode control register. Only on USARTs. */
		__I  uint32_t FIFOLVL;		/*!< FIFO Level register. Provides the current fill levels of the transmit and receive FIFOs. */
	};

	__IO uint32_t TER2;				/*!< Transmit Enable Register. Only on LPC177X_8X UART4 and LPC18XX/43XX USART0/2/3. */
} LPC_USART_T;
```
---
2. adcConfig( ADC_ENABLE );

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_adc.c` se encuentra la definición de `adcConfig`


```C
#define adcConfig adcInit
```

```C
void adcInit( adcInit_t config )
{
   /*
   Pines ADC EDU-CIAA-NXP

               pin  func
   ADC_CH1 ---- 2   ADC0_1/ADC1_1
   ADC_CH2 ---- 143 ADC0_2/ADC1_2
   ADC_CH3 ---- 139 ADC0_3/ADC1_3
   DAC     ---- 6   ADC0_0/ADC1_0/DAC

   T_FIL1  ---- 3   ADC0_1 (ANALOG_SEL)
   T_COL2  ---- 133 ADC0_3 (ANALOG_SEL)

   LCD1    ---- 9   DAC (ANALOG_SEL)

   T_FIL3  ---- 7   ADC0_0 (ANALOG_SEL)
   T_COL1  ---- 132 ADC0_4 (ANALOG_SEL)
   ENET_MDC --- 140 ADC1_6 (ANALOG_SEL)
   */

   switch(config) {

      case ADC_ENABLE: {

         /* Config ADC0 sample mode */      
         ADC_CLOCK_SETUP_T ADCSetup = {
            ADC_MAX_SAMPLE_RATE,   // ADC Sample rate:ADC_MAX_SAMPLE_RATE = 400KHz
            10,                    // ADC resolution: ADC_10BITS = 10
            0                      // ADC Burst Mode: (true or false)
         };

         Chip_ADC_Init( LPC_ADC0, &ADCSetup );
         /* Disable burst mode */
         Chip_ADC_SetBurstCmd( LPC_ADC0, DISABLE );
         /* Set sample rate to 200KHz */
         Chip_ADC_SetSampleRate( LPC_ADC0, &ADCSetup, ADC_MAX_SAMPLE_RATE/2 );
         /* Disable all channels */
         Chip_ADC_EnableChannel( LPC_ADC0, ADC_CH1, DISABLE );
         Chip_ADC_Int_SetChannelCmd( LPC_ADC0, ADC_CH1, DISABLE );

         Chip_ADC_EnableChannel( LPC_ADC0, ADC_CH2, DISABLE );
         Chip_ADC_Int_SetChannelCmd( LPC_ADC0, ADC_CH2, DISABLE );

         Chip_ADC_EnableChannel( LPC_ADC0, ADC_CH3, DISABLE );
         Chip_ADC_Int_SetChannelCmd( LPC_ADC0, ADC_CH3, DISABLE );

         Chip_ADC_EnableChannel( LPC_ADC0, ADC_CH4, DISABLE );
         Chip_ADC_Int_SetChannelCmd( LPC_ADC0, ADC_CH4, DISABLE );

         // For aditional ADC Inputs (Pablo Gomez)
         #if BOARD==edu_ciaa_nxp
         Chip_SCU_ADC_Channel_Config( 0, 4 );                      // Revisar codigo
         Chip_ADC_Int_SetChannelCmd( LPC_ADC0, ADC_CH5, DISABLE ); // Revisar codigo
         #endif
      }
      break;

      case ADC_DISABLE:
         /* Disable ADC peripheral */
         Chip_ADC_DeInit( LPC_ADC0 );
         break;
      }

}
```

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/inc/sapi_adc.h` se define la estructura `adcInit_t`:

```C
typedef enum {
   ADC_ENABLE, ADC_DISABLE
} adcInit_t;
```

Siguiendo con las funciones que se usan en la primitiva de `adcInit` se llega `Chip_ADC_Init` en `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/adc_18xx_43xx.c`:

```C
void Chip_ADC_Init(LPC_ADC_T *pADC, ADC_CLOCK_SETUP_T *ADCSetup)
{
	uint8_t div;
	uint32_t cr = 0;
	uint32_t clk;

	Chip_Clock_EnableOpts(Chip_ADC_GetClockIndex(pADC), true, true, 1);

	pADC->INTEN = 0;		/* Disable all interrupts */

	cr |= ADC_CR_PDN;
	ADCSetup->adcRate = ADC_MAX_SAMPLE_RATE;
	ADCSetup->bitsAccuracy = ADC_10BITS;
	clk = 11;
	ADCSetup->burstMode = false;
	div = getClkDiv(pADC, false, ADCSetup->adcRate, clk);
	cr |= ADC_CR_CLKDIV(div);
	cr |= ADC_CR_BITACC(ADCSetup->bitsAccuracy);
	pADC->CR = cr;
}
```
En `firmware_v3/libs/lpc_open/lpc_chip_43xx/src` en `adc_18xx_43xx.c ` se encuentra la función `Chip_ADC_SetBurstCmd`:

```C
/* Enable burst mode */
void Chip_ADC_SetBurstCmd(LPC_ADC_T *pADC, FunctionalState NewState)
{
	setStartMode(pADC, ADC_NO_START);

    if (NewState == DISABLE) {
		pADC->CR &= ~ADC_CR_BURST;
	}
	else {
		pADC->CR |= ADC_CR_BURST;
	}
}
```

En `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/adc_18xx_43xx.c`, también se encuentra la función `Chip_ADC_SetSampleRate`, `Chip_ADC_EnableChannel` y `Chip_ADC_Int_SetChannelCmd`:
```C
void Chip_ADC_SetSampleRate(LPC_ADC_T *pADC, ADC_CLOCK_SETUP_T *ADCSetup, uint32_t rate)
{
	uint8_t div;
	uint32_t cr;

	cr = pADC->CR & (~ADC_SAMPLE_RATE_CONFIG_MASK);
	ADCSetup->adcRate = rate;
	div = getClkDiv(pADC, ADCSetup->burstMode, rate, (11 - ADCSetup->bitsAccuracy));
	cr |= ADC_CR_CLKDIV(div);
	cr |= ADC_CR_BITACC(ADCSetup->bitsAccuracy);
	pADC->CR = cr;
}
```

```C
/* Enable or disable the ADC channel on ADC peripheral */
void Chip_ADC_EnableChannel(LPC_ADC_T *pADC, ADC_CHANNEL_T channel, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		pADC->CR |= ADC_CR_CH_SEL(channel);
	}
	else {
		pADC->CR &= ~ADC_CR_START_MASK;
		pADC->CR &= ~ADC_CR_CH_SEL(channel);
	}
}
```

```C
/* Enable/Disable interrupt for ADC channel */
void Chip_ADC_Int_SetChannelCmd(LPC_ADC_T *pADC, uint8_t channel, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		pADC->INTEN |= (1UL << channel);
	}
	else {
		pADC->INTEN &= (~(1UL << channel));
	}
}
```

En la estructura `LPC_ADC_T` se ve la descripción de los registros que se usan en las funciones detalladas anteriormente:

```C
 /* @brief 10 or 12-bit ADC register block structure
 */
typedef struct {					/*!< ADCn Structure */
	__IO uint32_t CR;				/*!< A/D Control Register. The AD0CR register must be written to select the operating mode before A/D conversion can occur. */
	__I  uint32_t GDR;				/*!< A/D Global Data Register. Contains the result of the most recent A/D conversion. */
	__I  uint32_t RESERVED0;
	__IO uint32_t INTEN;			/*!< A/D Interrupt Enable Register. This register contains enable bits that allow the DONE flag of each A/D channel to be included or excluded from contributing to the generation of an A/D interrupt. */
	__I  uint32_t DR[8];			/*!< A/D Channel Data Register. This register contains the result of the most recent conversion completed on channel n. */
	__I  uint32_t STAT;				/*!< A/D Status Register. This register contains DONE and OVERRUN flags for all of the A/D channels, as well as the A/D interrupt flag. */
} LPC_ADC_T;
```
---
3. dacConfig( DAC_ENABLE );

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_dac.c` se encuentra la definición de `dacConfig`


```C
#define dacConfig dacInit
```

```C
void dacInit( dacInit_t config )
{

   switch(config) {

   case DAC_ENABLE:
      /* Initialize the DAC peripheral */
      //Chip_DAC_Init(LPC_DAC);
      Chip_Clock_EnableOpts(CLK_APB3_DAC, true, true, 1);
      /* Set update rate to 400 KHz */
      Chip_DAC_SetBias(LPC_DAC, DAC_MAX_UPDATE_RATE_400kHz);

      /* Enables the DMA operation and controls DMA timer */
      Chip_DAC_ConfigDAConverterControl(LPC_DAC, DAC_DMA_ENA);
      /* DCAR DMA access */
      /* Update value to DAC buffer*/
      Chip_DAC_UpdateValue(LPC_DAC, 0);
      break;

   case DAC_DISABLE:
      /* Disable DAC peripheral */
      Chip_DAC_DeInit( LPC_DAC );
      break;
   }
}
```
En `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/clock_18xx_43xx.c` se encuentra la función `Chip_Clock_EnableOpts`:

```C
/* Enables a peripheral clock and sets clock states */
void Chip_Clock_EnableOpts(CHIP_CCU_CLK_T clk, bool autoen, bool wakeupen, int div)
{
	uint32_t reg = 1;

	if (autoen) {
		reg |= (1 << 1);
	}
	if (wakeupen) {
		reg |= (1 << 2);
	}

	/* Not all clocks support a divider, but we won't check that here. Only
	   dividers of 1 and 2 are allowed. Assume 1 if not 2 */
	if (div == 2) {
		reg |= (1 << 5);
	}

	/* Setup peripheral clock and start running */
	if (clk >= CLK_CCU2_START) {
		LPC_CCU2->CLKCCU[clk - CLK_CCU2_START].CFG = reg;
	}
	else {
		LPC_CCU1->CLKCCU[clk].CFG = reg;
	}
}
```

En `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/dac_18xx_43xx.c` se encuentra la función `Chip_DAC_SetBias`:

```C
/* Set Maximum update rate for DAC */
void Chip_DAC_SetBias(LPC_DAC_T *pDAC, uint32_t bias)
{
	pDAC->CR &= ~DAC_BIAS_EN;

	if (bias  == DAC_MAX_UPDATE_RATE_400kHz) {
		pDAC->CR |= DAC_BIAS_EN;
	}
}
```

En `firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/dac_18xx_43xx.h` se encuentra la definición de `firmware_v3/libs/lpc_open/lpc_chip_43xx/inc$ ls dac_18xx_43xx.h`:

```C
/**
 * @brief	Enables the DMA operation and controls DMA timer
 * @param	pDAC		: pointer to LPC_DAC_T
 * @param	dacFlags	: An Or'ed value of the following DAC values:
 *                  - DAC_DBLBUF_ENA :enable/disable DACR double buffering feature
 *                  - DAC_CNT_ENA    :enable/disable timer out counter
 *                  - DAC_DMA_ENA    :enable/disable DMA access
 * @return	Nothing
 * @note	Pass an Or'ed value of the DAC flags to enable those options.
 */
STATIC INLINE void Chip_DAC_ConfigDAConverterControl(LPC_DAC_T *pDAC, uint32_t dacFlags)
{
	uint32_t temp;

	temp = pDAC->CTRL & ~DAC_DACCTRL_MASK;
	pDAC->CTRL = temp | dacFlags;
}
```

En `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/dac_18xx_43xx.c` se encuentra la función `Chip_DAC_UpdateValue`:

```C
/* Update value to DAC buffer*/
void Chip_DAC_UpdateValue(LPC_DAC_T *pDAC, uint32_t dac_value)
{
	uint32_t tmp;

	tmp = pDAC->CR & DAC_BIAS_EN;
	tmp |= DAC_VALUE(dac_value);
	/* Update value */
	pDAC->CR = tmp;
}
```
En `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/dac_18xx_43xx.c` se encuentra la función `Chip_DAC_DeInit`:

```C
/* Shutdown DAC peripheral */
void Chip_DAC_DeInit(LPC_DAC_T *pDAC)
{
	Chip_Clock_Disable(CLK_APB3_DAC);
}
```
En `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/clock_18xx_43xx.c` se encuentra la función `Chip_Clock_Disable`:

```C
/* Disables a peripheral clock */
void Chip_Clock_Disable(CHIP_CCU_CLK_T clk)
{
	/* Stop peripheral clock */
	if (clk >= CLK_CCU2_START) {
		LPC_CCU2->CLKCCU[clk - CLK_CCU2_START].CFG &= ~1;
	}
	else {
		LPC_CCU1->CLKCCU[clk].CFG &= ~1;
	}
}
```

Los resgitros `CR`, `CTRL` y `CNTVAL`, se definen en la estructura `LPC_DAC_T` en `/firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/dac_18xx_43xx.h`

```
/**
 */ @brief DAC register block structure
 */
typedef struct {			/*!< DAC Structure          */
	__IO uint32_t  CR;		/*!< DAC register. Holds the conversion data. */
	__IO uint32_t  CTRL;	/*!< DAC control register.  */
	__IO uint32_t  CNTVAL;	/*!< DAC counter value register. */
} LPC_DAC_T;
```

En las funciones que se encuentran en `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/clock_18xx_43xx.c` se utiliza una estructura con registros del clock `CHIP_CCU_CLK_T`. Dicha estructura se define en `firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/chip_clocks.h `
```
typedef enum CHIP_CCU_CLK {
	/* CCU1 clocks */
	CLK_APB3_BUS,		/*!< APB3 bus clock from base clock CLK_BASE_APB3 */
	CLK_APB3_I2C1,		/*!< I2C1 register/perigheral clock from base clock CLK_BASE_APB3 */
	CLK_APB3_DAC,		/*!< DAC peripheral clock from base clock CLK_BASE_APB3 */
	CLK_APB3_ADC0,		/*!< ADC0 register/perigheral clock from base clock CLK_BASE_APB3 */
	CLK_APB3_ADC1,		/*!< ADC1 register/perigheral clock from base clock CLK_BASE_APB3 */
	CLK_APB3_CAN0,		/*!< CAN0 register/perigheral clock from base clock CLK_BASE_APB3 */
	CLK_APB1_BUS = 32,	/*!< APB1 bus clock clock from base clock CLK_BASE_APB1 */
	CLK_APB1_MOTOCON,	/*!< Motor controller register/perigheral clock from base clock CLK_BASE_APB1 */
	CLK_APB1_I2C0,		/*!< I2C0 register/perigheral clock from base clock CLK_BASE_APB1 */
	CLK_APB1_I2S,		/*!< I2S register/perigheral clock from base clock CLK_BASE_APB1 */
	CLK_APB1_CAN1,		/*!< CAN1 register/perigheral clock from base clock CLK_BASE_APB1 */
	CLK_SPIFI = 64,		/*!< SPIFI SCKI input clock from base clock CLK_BASE_SPIFI */
	CLK_MX_BUS = 96,	/*!< M3/M4 BUS core clock from base clock CLK_BASE_MX */
	CLK_MX_SPIFI,		/*!< SPIFI register clock from base clock CLK_BASE_MX */
	CLK_MX_GPIO,		/*!< GPIO register clock from base clock CLK_BASE_MX */
	CLK_MX_LCD,			/*!< LCD register clock from base clock CLK_BASE_MX */
	CLK_MX_ETHERNET,	/*!< ETHERNET register clock from base clock CLK_BASE_MX */
	CLK_MX_USB0,		/*!< USB0 register clock from base clock CLK_BASE_MX */
	CLK_MX_EMC,			/*!< EMC clock from base clock CLK_BASE_MX */
	CLK_MX_SDIO,		/*!< SDIO register clock from base clock CLK_BASE_MX */
	CLK_MX_DMA,			/*!< DMA register clock from base clock CLK_BASE_MX */
	CLK_MX_MXCORE,		/*!< M3/M4 CPU core clock from base clock CLK_BASE_MX */
	RESERVED_ALIGN = CLK_MX_MXCORE + 3,
	CLK_MX_SCT,			/*!< SCT register clock from base clock CLK_BASE_MX */
	CLK_MX_USB1,		/*!< USB1 register clock from base clock CLK_BASE_MX */
	CLK_MX_EMC_DIV,		/*!< ENC divider clock from base clock CLK_BASE_MX */
	CLK_MX_FLASHA,		/*!< FLASHA bank clock from base clock CLK_BASE_MX */
	CLK_MX_FLASHB,		/*!< FLASHB bank clock from base clock CLK_BASE_MX */
#if defined(CHIP_LPC43XX)
	CLK_M4_M0APP,		/*!< M0 app CPU core clock from base clock CLK_BASE_MX */
	CLK_MX_ADCHS,		/*!< ADCHS clock from base clock CLK_BASE_ADCHS */
#else
	CLK_RESERVED1,
	CLK_RESERVED2,
#endif
	CLK_MX_EEPROM,		/*!< EEPROM clock from base clock CLK_BASE_MX */
	CLK_MX_WWDT = 128,	/*!< WWDT register clock from base clock CLK_BASE_MX */
	CLK_MX_UART0,		/*!< UART0 register clock from base clock CLK_BASE_MX */
	CLK_MX_UART1,		/*!< UART1 register clock from base clock CLK_BASE_MX */
	CLK_MX_SSP0,		/*!< SSP0 register clock from base clock CLK_BASE_MX */
	CLK_MX_TIMER0,		/*!< TIMER0 register/perigheral clock from base clock CLK_BASE_MX */
	CLK_MX_TIMER1,		/*!< TIMER1 register/perigheral clock from base clock CLK_BASE_MX */
	CLK_MX_SCU,			/*!< SCU register/perigheral clock from base clock CLK_BASE_MX */
	CLK_MX_CREG,		/*!< CREG clock from base clock CLK_BASE_MX */
	CLK_MX_RITIMER = 160,	/*!< RITIMER register/perigheral clock from base clock CLK_BASE_MX */
	CLK_MX_UART2,		/*!< UART3 register clock from base clock CLK_BASE_MX */
	CLK_MX_UART3,		/*!< UART4 register clock from base clock CLK_BASE_MX */
	CLK_MX_TIMER2,		/*!< TIMER2 register/perigheral clock from base clock CLK_BASE_MX */
	CLK_MX_TIMER3,		/*!< TIMER3 register/perigheral clock from base clock CLK_BASE_MX */
	CLK_MX_SSP1,		/*!< SSP1 register clock from base clock CLK_BASE_MX */
	CLK_MX_QEI,			/*!< QEI register/perigheral clock from base clock CLK_BASE_MX */
#if defined(CHIP_LPC43XX)
	CLK_PERIPH_BUS = 192,	/*!< Peripheral bus clock from base clock CLK_BASE_PERIPH */
	CLK_RESERVED3,
	CLK_PERIPH_CORE,	/*!< Peripheral core clock from base clock CLK_BASE_PERIPH */
	CLK_PERIPH_SGPIO,	/*!< SGPIO clock from base clock CLK_BASE_PERIPH */
#else
	CLK_RESERVED3 = 192,
	CLK_RESERVED3A,
	CLK_RESERVED4,
	CLK_RESERVED5,
#endif
	CLK_USB0 = 224,			/*!< USB0 clock from base clock CLK_BASE_USB0 */
	CLK_USB1 = 256,			/*!< USB1 clock from base clock CLK_BASE_USB1 */
#if defined(CHIP_LPC43XX)
	CLK_SPI = 288,			/*!< SPI clock from base clock CLK_BASE_SPI */
	CLK_ADCHS = 320,		/*!< ADCHS clock from base clock CLK_BASE_ADCHS */
#else
	CLK_RESERVED7 = 320,
	CLK_RESERVED8,
#endif
	CLK_CCU1_LAST,

	/* CCU2 clocks */
	CLK_CCU2_START,
	CLK_APLL = CLK_CCU2_START,	/*!< Audio PLL clock from base clock CLK_BASE_APLL */
	RESERVED_ALIGNB = CLK_CCU2_START + 31,
	CLK_APB2_UART3,			/*!< UART3 clock from base clock CLK_BASE_UART3 */
	RESERVED_ALIGNC = CLK_CCU2_START + 63,
	CLK_APB2_UART2,			/*!< UART2 clock from base clock CLK_BASE_UART2 */
	RESERVED_ALIGND = CLK_CCU2_START + 95,
	CLK_APB0_UART1,			/*!< UART1 clock from base clock CLK_BASE_UART1 */
	RESERVED_ALIGNE = CLK_CCU2_START + 127,
	CLK_APB0_UART0,			/*!< UART0 clock from base clock CLK_BASE_UART0 */
	RESERVED_ALIGNF = CLK_CCU2_START + 159,
	CLK_APB2_SSP1,			/*!< SSP1 clock from base clock CLK_BASE_SSP1 */
	RESERVED_ALIGNG = CLK_CCU2_START + 191,
	CLK_APB0_SSP0,			/*!< SSP0 clock from base clock CLK_BASE_SSP0 */
	RESERVED_ALIGNH = CLK_CCU2_START + 223,
	CLK_APB2_SDIO,			/*!< SDIO clock from base clock CLK_BASE_SDIO */
	CLK_CCU2_LAST
} CHIP_CCU_CLK_T;
```
---

4. delayConfig( &delay, 500 );

En `firmware_v3/libs/sapi/sapi_v0.5.2/abstract_modules/inc/sapi_delay.h` se define `delayConfig`:

```C
#define delayConfig delayInit
```

```C
void delayInit( delay_t * delay, tick_t duration )
{
   delay->duration = duration/tickRateMS;
   delay->running = 0;
}
```
Las estructuras `delay_t` y `tick_t` se definen en `firmware_v3/libs/sapi/sapi_v0.5.2/abstract_modules/inc/sapi_delay.h ` y `firmware_v3/libs/sapi/sapi_v0.5.2/base/inc/sapi_datatypes.h` respectivamente:

```
typedef struct{
   tick_t startTime;
   tick_t duration;
   bool_t running;
} delay_t;
```
```
/* Define Tick Data Type */
typedef uint64_t tick_t;
```

Además en la función `delayInit` se usa la variable `tickRateMS`, que se define en `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_tick.c`:

```
tick_t tickRateMS = 1; // Used by delay!!! Default 1ms

```
---
5. muestra = adcRead( CH1 );

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_adc.c` se encuentra la definición de `adcRead`


```C
uint16_t adcRead( adcMap_t analogInput )
{
   uint8_t lpcAdcChannel = analogInput + 1;
   uint16_t analogValue = 0;

   // Enable channel
   Chip_ADC_EnableChannel(LPC_ADC0, lpcAdcChannel, ENABLE);

   // Start conversion
   Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

   // Wait for conversion complete
   while(
      (Chip_ADC_ReadStatus(LPC_ADC0, lpcAdcChannel, ADC_DR_DONE_STAT) != SET)
   );

   // Enable Read value
   Chip_ADC_ReadValue( LPC_ADC0, lpcAdcChannel, &analogValue );

   // Disable channel
   Chip_ADC_EnableChannel( LPC_ADC0, lpcAdcChannel, DISABLE );

   return analogValue;
}
```

En `firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/adc_18xx_43xx.h` se define también:

```
/**
 * @brief	Select the mode starting the AD conversion
 * @param	pADC		: The base of ADC peripheral on the chip
 * @param	mode		: Stating mode, should be :
 *							- ADC_NO_START				: Must be set for Burst mode
 *							- ADC_START_NOW				: Start conversion now
 *							- ADC_START_ON_CTOUT15		: Start conversion when the edge selected by bit 27 occurs on CTOUT_15
 *							- ADC_START_ON_CTOUT8		: Start conversion when the edge selected by bit 27 occurs on CTOUT_8
 *							- ADC_START_ON_ADCTRIG0		: Start conversion when the edge selected by bit 27 occurs on ADCTRIG0
 *							- ADC_START_ON_ADCTRIG1		: Start conversion when the edge selected by bit 27 occurs on ADCTRIG1
 *							- ADC_START_ON_MCOA2		: Start conversion when the edge selected by bit 27 occurs on Motocon PWM output MCOA2
 * @param	EdgeOption	: Stating Edge Condition, should be :
 *							- ADC_TRIGGERMODE_RISING	: Trigger event on rising edge
 *							- ADC_TRIGGERMODE_FALLING	: Trigger event on falling edge
 * @return	Nothing
 */
void Chip_ADC_SetStartMode(LPC_ADC_T *pADC, ADC_START_MODE_T mode, ADC_EDGE_CFG_T EdgeOption);
```


 En `firmware_v3/libs/sapi/sapi_v0.5.2/board/inc/sapi_peripheral_map.h` se define `adcMap_t`:

```
/* Defined for sapi_adc.h */
typedef enum {
	#if (BOARD == ciaa_nxp)
	   AI0 = 0, // AIN0 =   2 ADC0_1/ADC1_1
	   AI1 = 1, // AIN1 = 143 ADC0_2/ADC1_2
	   AI2 = 2, // AIN2 = 139 ADC0_3/ADC1_3
	   AI3 = 3, // AIN3 = 138 ADC0_4/ADC1_4
	#elif (BOARD == edu_ciaa_nxp)
	   CH1 = 0, // CH1 =   2 ADC0_1/ADC1_1
	   CH2 = 1, // CH2 = 143 ADC0_2/ADC1_2
	   CH3 = 2, // CH3 = 139 ADC0_3/ADC1_3
	#else
	   #error BOARD not supported yet!
	#endif
} adcMap_t;
```
Y se vuelve a llamar a las funciones `Chip_ADC_EnableChannel` y `Chip_ADC_EnableChannel` que se describieron en el punto `2.`, donde se expliaca la primitiva de  `adcConfig`.

Además, en la primitiva `adcRead` se llama a las siguientes funciones:

En `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/adc_18xx_43xx.c` se define `Chip_ADC_SetStartMode`

```
/* Select the mode starting the AD conversion */
void Chip_ADC_SetStartMode(LPC_ADC_T *pADC, ADC_START_MODE_T mode, ADC_EDGE_CFG_T EdgeOption)
{
	if ((mode != ADC_START_NOW) && (mode != ADC_NO_START)) {
		if (EdgeOption) {
			pADC->CR |= ADC_CR_EDGE;
		}
		else {
			pADC->CR &= ~ADC_CR_EDGE;
		}
	}
	setStartMode(pADC, (uint8_t) mode);
}
```
La estructura de registros `LPC_ADC_T` es la misma que se muestra en el punto `2.` donde se define `adcConfig` y las estructuras `ADC_START_MODE_T` y `ADC_EDGE_CFG_T` que se definen en `firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/adc_18xx_43xx.h`:

```
/** Start mode, which controls the start of an A/D conversion when the BURST bit is 0. */
typedef enum CHIP_ADC_START_MODE {
	ADC_NO_START = 0,
	ADC_START_NOW,			/*!< Start conversion now */
	ADC_START_ON_CTOUT15,	/*!< Start conversion when the edge selected by bit 27 occurs on CTOUT_15 */
	ADC_START_ON_CTOUT8,	/*!< Start conversion when the edge selected by bit 27 occurs on CTOUT_8 */
	ADC_START_ON_ADCTRIG0,	/*!< Start conversion when the edge selected by bit 27 occurs on ADCTRIG0 */
	ADC_START_ON_ADCTRIG1,	/*!< Start conversion when the edge selected by bit 27 occurs on ADCTRIG1 */
	ADC_START_ON_MCOA2		/*!< Start conversion when the edge selected by bit 27 occurs on Motocon PWM output MCOA2 */
} ADC_START_MODE_T;
```
```
/** Edge configuration, which controls rising or falling edge on the selected signal for the start of a conversion */
typedef enum CHIP_ADC_EDGE_CFG {
	ADC_TRIGGERMODE_RISING = 0,		/**< Trigger event: rising edge */
	ADC_TRIGGERMODE_FALLING,		/**< Trigger event: falling edge */
} ADC_EDGE_CFG_T;
```
En `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/adc_18xx_43xx.c` se define `Chip_ADC_ReadStatus` y `Chip_ADC_ReadValue`:


```
/* Get ADC Channel status from ADC data register */
FlagStatus Chip_ADC_ReadStatus(LPC_ADC_T *pADC, uint8_t channel, uint32_t StatusType)
{
	switch (StatusType) {
	case ADC_DR_DONE_STAT:
		return (pADC->STAT & (1UL << channel)) ? SET : RESET;

	case ADC_DR_OVERRUN_STAT:
		channel += 8;
		return (pADC->STAT & (1UL << channel)) ? SET : RESET;

	case ADC_DR_ADINT_STAT:
		return pADC->STAT >> 16 ? SET : RESET;

	default:
		break;
	}
	return RESET;
}
```

```
/* Get the ADC value */
Status Chip_ADC_ReadValue(LPC_ADC_T *pADC, uint8_t channel, uint16_t *data)
{
	return readAdcVal(pADC, channel, data);
}
```

y luego se define `readAdcVal`:

```
/* Get the ADC value */
Status readAdcVal(LPC_ADC_T *pADC, uint8_t channel, uint16_t *data)
{
	uint32_t temp;
	temp = pADC->DR[channel];
	if (!ADC_DR_DONE(temp)) {
		return ERROR;
	}
	/*	if(ADC_DR_OVERRUN(temp) && (pADC->CR & ADC_CR_BURST)) */
	/*	return ERROR; */
	*data = (uint16_t) ADC_DR_RESULT(temp);
	return SUCCESS;
}
```

---

6. uartReadByte( UART_USB, &dato )

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_uart.c` se encuentra la definición de `uartReadByte` :

```C
// Read 1 byte from RX FIFO, check first if exist aviable data
bool_t uartReadByte( uartMap_t uart, uint8_t* receivedByte )
{
   bool_t retVal = TRUE;
   if ( uartRxReady(uart) ) {
      *receivedByte = uartRxRead(uart);
   } else {
      retVal = FALSE;
   }
   return retVal;
}
```
La estructura `uartMap_t` se describe en el punto `6.` cuando se explicó la primitiva `uartConfig`.
---
7. uartWriteByte( UART_USB, dato )

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_uart.c` se encuentra la definición de `uartWriteByte`:

```C
void uartWriteByte( uartMap_t uart, const uint8_t value )
{
   // Wait for space in FIFO (blocking)
   while( uartTxReady( uart ) == FALSE );
   // Send byte
   uartTxWrite( uart, value );
}
```

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_uart.c` también se encuentra la definición de `uartTxReady` y `uartTxWrite`:

```C
// Return TRUE if have space in TX FIFO
bool_t uartTxReady( uartMap_t uart )
{
   return Chip_UART_ReadLineStatus( lpcUarts[uart].uartAddr ) & UART_LSR_THRE;
}
```

```
// Write in TX FIFO
void uartTxWrite( uartMap_t uart, const uint8_t value )
{
   Chip_UART_SendByte( lpcUarts[uart].uartAddr, value );
}
```

La funciones `Chip_UART_ReadLineStatus` y `Chip_UART_SendByte` y se definen en `/firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/uart_18xx_43xx.h`:

```
/**
 * @brief	Return Line Status register/status (LSR)
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	Line Status register (status)
 * @note	Mask bits of the returned status value with UART_LSR_*
 *			definitions for specific statuses.
 */
STATIC INLINE uint32_t Chip_UART_ReadLineStatus(LPC_USART_T *pUART)
{
	return pUART->LSR;
}
```

```
STATIC INLINE void Chip_UART_SendByte(LPC_USART_T *pUART, uint8_t data)
{
	pUART->THR = (uint32_t) data;
}

```

También en `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_uart.c` se muestra como se define el array `lpcUarts`:

```
typedef struct {
   LPC_USART_T*      uartAddr;
   lpc4337ScuPin_t   txPin;
   lpc4337ScuPin_t   rxPin;
   IRQn_Type         uartIrqAddr;
} uartLpcInit_t;

/*==================[internal data declaration]==============================*/

#ifdef SAPI_USE_INTERRUPTS
static callBackFuncPtr_t rxIsrCallbackUART0 = 0;
static void* rxIsrCallbackUART0Params = NULL;
static callBackFuncPtr_t rxIsrCallbackUART2 = 0;
static void* rxIsrCallbackUART2Params = NULL;
static callBackFuncPtr_t rxIsrCallbackUART3 = 0;
static void* rxIsrCallbackUART3Params = NULL;

static callBackFuncPtr_t txIsrCallbackUART0 = 0;
static void* txIsrCallbackUART0Params = NULL;
static callBackFuncPtr_t txIsrCallbackUART2 = 0;
static void* txIsrCallbackUART2Params = NULL;
static callBackFuncPtr_t txIsrCallbackUART3 = 0;
static void* txIsrCallbackUART3Params = NULL;
#endif /* SAPI_USE_INTERRUPTS */

static const uartLpcInit_t lpcUarts[] = {
// { uartAddr, { txPort, txpin, txfunc }, { rxPort, rxpin, rxfunc }, uartIrqAddr  },
   // UART_GPIO (GPIO1 = U0_TXD, GPIO2 = U0_RXD)
   { LPC_USART0, { 6, 4, FUNC2 }, { 6, 5, FUNC2 }, USART0_IRQn }, // 0
   // UART_485 (RS485/Profibus)
   { LPC_USART0, { 9, 5, FUNC7 }, { 9, 6, FUNC7 }, USART0_IRQn }, // 1
   // UART not routed
   {  LPC_UART1, { 0, 0, 0     }, { 0, 0, 0     }, UART1_IRQn  }, // 2
   // UART_USB
   { LPC_USART2, { 7, 1, FUNC6 }, { 7, 2, FUNC6 }, USART2_IRQn }, // 3
   // UART_ENET
   { LPC_USART2, { 1,15, FUNC1 }, { 1,16, FUNC1 }, USART2_IRQn }, // 4
   // UART_232
   { LPC_USART3, { 2, 3, FUNC2 }, { 2, 4, FUNC2 }, USART3_IRQn }  // 5
};
```

Además en `firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/uart_18xx_43xx.h` se define  el registro `UART_RBR_MASKBIT`:

```
/**
 * @brief Macro defines for UART Receive Buffer register
 */
#define UART_RBR_MASKBIT    (0xFF)		        /*!< UART Received Buffer mask bit (8 bits) */
```

En la estructura `LPC_USART_T` están definidos los registros `LSR` y `THR` que se usan en `Chip_UART_ReadLineStatus` y `Chip_UART_SendByte` respectivamente:

```
typedef struct {					/*!< USARTn Structure       */

	union {
		__IO uint32_t  DLL;			/*!< Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider (DLAB = 1). */
		__O  uint32_t  THR;			/*!< Transmit Holding Register. The next character to be transmitted is written here (DLAB = 0). */
		__I  uint32_t  RBR;			/*!< Receiver Buffer Register. Contains the next received character to be read (DLAB = 0). */
	};

	union {
		__IO uint32_t IER;			/*!< Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential UART interrupts (DLAB = 0). */
		__IO uint32_t DLM;			/*!< Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider (DLAB = 1). */
	};

	union {
		__O  uint32_t FCR;			/*!< FIFO Control Register. Controls UART FIFO usage and modes. */
		__I  uint32_t IIR;			/*!< Interrupt ID Register. Identifies which interrupt(s) are pending. */
	};

	__IO uint32_t LCR;				/*!< Line Control Register. Contains controls for frame formatting and break generation. */
	__IO uint32_t MCR;				/*!< Modem Control Register. Only present on USART ports with full modem support. */
	__I  uint32_t LSR;				/*!< Line Status Register. Contains flags for transmit and receive status, including line errors. */
	__I  uint32_t MSR;				/*!< Modem Status Register. Only present on USART ports with full modem support. */
	__IO uint32_t SCR;				/*!< Scratch Pad Register. Eight-bit temporary storage for software. */
	__IO uint32_t ACR;				/*!< Auto-baud Control Register. Contains controls for the auto-baud feature. */
	__IO uint32_t ICR;				/*!< IrDA control register (not all UARTS) */
	__IO uint32_t FDR;				/*!< Fractional Divider Register. Generates a clock input for the baud rate divider. */
	__IO uint32_t OSR;				/*!< Oversampling Register. Controls the degree of oversampling during each bit time. Only on some UARTS. */
	__IO uint32_t TER1;				/*!< Transmit Enable Register. Turns off USART transmitter for use with software flow control. */
	uint32_t  RESERVED0[3];
    __IO uint32_t HDEN;				/*!< Half-duplex enable Register- only on some UARTs */
	__I  uint32_t RESERVED1[1];
	__IO uint32_t SCICTRL;			/*!< Smart card interface control register- only on some UARTs */

	__IO uint32_t RS485CTRL;		/*!< RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes. */
	__IO uint32_t RS485ADRMATCH;	/*!< RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode. */
	__IO uint32_t RS485DLY;			/*!< RS-485/EIA-485 direction control delay. */

	union {
		__IO uint32_t SYNCCTRL;		/*!< Synchronous mode control register. Only on USARTs. */
		__I  uint32_t FIFOLVL;		/*!< FIFO Level register. Provides the current fill levels of the transmit and receive FIFOs. */
	};

	__IO uint32_t TER2;				/*!< Transmit Enable Register. Only on LPC177X_8X UART4 and LPC18XX/43XX USART0/2/3. */
} LPC_USART_T;
```


---

8. uartWriteString( UART_USB, "ADC CH1 value: " );

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_uart.c`  se define la primitiva `uartWriteString`:

Que llama a la función `uartWriteByte` que es la definida en el punto `7.`, mientras la variable `str` contenga algo:

```C
void uartWriteString( uartMap_t uart, const char* str )
{
   while( *str != 0 ) {
      uartWriteByte( uart, (uint8_t)*str );
      str++;
   }
}
```


---
9. dacWrite( DAC, muestra );

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_uart.c` se define la primitiva `dacWrite`:

```C
void dacWrite( dacMap_t analogOutput, uint16_t value )
{
   if( analogOutput == 0 ) {
      if( value > 1023 ) {
         value = 1023;
      }
      Chip_DAC_UpdateValue( LPC_DAC, value );
   }
}
```

En la estructura `dacMap_t` se determina cual es la placa que se está utilizando, para mapera los registros, en este caso `DAC` y `DAC0`:

```C
/* Defined for sapi_dac.h */
typedef enum {
	#if (BOARD == ciaa_nxp)
		AO  = 0,
		AO0 = 0,
	#elif (BOARD == edu_ciaa_nxp)
		DAC  = 0,
		DAC0 = 0,
	#else
	   #error BOARD not supported yet!
	#endif
} dacMap_t;
```

La función `Chip_DAC_UpdateValue` que se llama para actualizar el valor de salida, se define en `firmware_v3/libs/lpc_open/lpc_chip_43xx/src/dac_18xx_43xx.c` :

```C
/* Update value to DAC buffer*/
void Chip_DAC_UpdateValue(LPC_DAC_T *pDAC, uint32_t dac_value)
{
	uint32_t tmp;

	tmp = pDAC->CR & DAC_BIAS_EN;
	tmp |= DAC_VALUE(dac_value);
	/* Update value */
	pDAC->CR = tmp;
}
```


En `firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/dac_18xx_43xx.h` se definen `DAC_VALUE` y `DAC_BIAS_EN`:

```C
/** After the selected settling time after this field is written with a
   new VALUE, the voltage on the AOUT pin (with respect to VSSA)
   is VALUE/1024 ? VREF */
#define DAC_VALUE(n)        ((uint32_t) ((n & 0x3FF) << 6))
/** If this bit = 0: The settling time of the DAC is 1 microsecond max,
 * and the maximum current is 700 microAmpere
 * If this bit = 1: The settling time of the DAC is 2.5 microsecond
 * and the maximum current is 350 microAmpere
 */
#define DAC_BIAS_EN         ((uint32_t) (1 << 16))
```

---

10. uartCallbackSet(UART_USB, UART_RECEIVE, onRx, NULL);

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_uart.c` se encuentra la definición de `uartCallbackSet`:

```C
// UART Interrupt event Enable and set a callback
void uartCallbackSet( uartMap_t uart, uartEvents_t event,
                      callBackFuncPtr_t callbackFunc, void* callbackParam )
{   
   uint32_t intMask;

   switch(event){

      case UART_RECEIVE:
         // Enable UART Receiver Buffer Register Interrupt
         //intMask = UART_IER_RBRINT;

         // Enable UART Receiver Buffer Register Interrupt and Enable UART line
         //status interrupt. LPC43xx User manual page 1118
         intMask = UART_IER_RBRINT | UART_IER_RLSINT;

         if( callbackFunc != 0 ) {
            // Set callback
            if( (uart == UART_GPIO) || (uart == UART_485) ){
               rxIsrCallbackUART0 = callbackFunc;
               rxIsrCallbackUART0Params = callbackParam;
            }
            if( (uart == UART_USB) || (uart == UART_ENET) ){
               rxIsrCallbackUART2 = callbackFunc;
               rxIsrCallbackUART2Params = callbackParam;
            }            
            if( uart == UART_232 ){
               rxIsrCallbackUART3 = callbackFunc;
               rxIsrCallbackUART3Params = callbackParam;
            }
         } else{
            return;
         }
      break;

      case UART_TRANSMITER_FREE:
         // Enable THRE irq (TX)
         intMask = UART_IER_THREINT;

         if( callbackFunc != 0 ) {

            // Set callback
            if( (uart == UART_GPIO) || (uart == UART_485) ){
               txIsrCallbackUART0 = callbackFunc;
               txIsrCallbackUART0Params = callbackParam;
            }
            if( (uart == UART_USB) || (uart == UART_ENET) ){
               txIsrCallbackUART2 = callbackFunc;
               txIsrCallbackUART2Params = callbackParam;
            }            
            if( uart == UART_232 ){
               txIsrCallbackUART3 = callbackFunc;
               txIsrCallbackUART3Params = callbackParam;
            }
         } else{
            return;
         }
      break;

      default:
         return;
   }

   // Enable UART Interrupt
   Chip_UART_IntEnable(lpcUarts[uart].uartAddr, intMask);
}
```

La estructura `uartEvents_t` se define en `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/inc/sapi_uart.h`:

```C
typedef enum{
   UART_RECEIVE,
   UART_TRANSMITER_FREE
} uartEvents_t;
```
Los registros `UART_IER_RBRINT` y `UART_IER_RLSINT` , están definidos en `/firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/uart_18xx_43xx.h`:

```
/**
 * @brief Macro defines for UART Interrupt Enable Register
 */
#define UART_IER_RBRINT      (1 << 0)	/*!< RBR Interrupt enable */
#define UART_IER_THREINT     (1 << 1)	/*!< THR Interrupt enable */
#define UART_IER_RLSINT      (1 << 2)	/*!< RX line status interrupt enable */
#define UART_IER_MSINT       (1 << 3)	/*!< Modem status interrupt enable - valid for 11xx, 17xx/40xx UART1, 18xx/43xx UART1  only */
#define UART_IER_CTSINT      (1 << 7)	/*!< CTS signal transition interrupt enable - valid for 17xx/40xx UART1, 18xx/43xx UART1 only */
#define UART_IER_ABEOINT     (1 << 8)	/*!< Enables the end of auto-baud interrupt */
#define UART_IER_ABTOINT     (1 << 9)	/*!< Enables the auto-baud time-out interrupt */
#define UART_IER_BITMASK     (0x307)	/*!< UART interrupt enable register bit mask  - valid for 13xx, 17xx/40xx UART0/2/3, 18xx/43xx UART0/2/3 only*/
#define UART1_IER_BITMASK    (0x30F)	/*!< UART1 interrupt enable register bit mask - valid for 11xx only */
#define UART2_IER_BITMASK    (0x38F)	/*!< UART2 interrupt enable register bit mask - valid for 17xx/40xx UART1, 18xx/43xx UART1 only */
```

---

11. uartInterrupt(UART_USB, true);

En `firmware_v3/libs/sapi/sapi_v0.5.2/soc/peripherals/src/sapi_uart.c` se encuentra la definición de `uartInterrupt`:

```C
// UART Global Interrupt Enable/Disable
void uartInterrupt( uartMap_t uart, bool_t enable )
{
   if( enable ) {
      // Interrupt Priority for UART channel
      NVIC_SetPriority( lpcUarts[uart].uartIrqAddr, 5 ); // FreeRTOS Requiere prioridad >= 5 (numero mas alto, mas baja prioridad)
      // Enable Interrupt for UART channel
      NVIC_EnableIRQ( lpcUarts[uart].uartIrqAddr );
   } else {
      // Disable Interrupt for UART channel
      NVIC_DisableIRQ( lpcUarts[uart].uartIrqAddr );
   }
}
```
