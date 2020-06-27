

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

También se habilita la trasnmisión en el pin Tx en `/firmware_v3/libs/lpc_open/lpc_chip_43xx/inc/uart_18xx_43xx.h `:

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
