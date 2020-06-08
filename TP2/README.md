# Trabajo Práctico nro. 2 - LPC43xx Entradas y Salidas (Digitales) de Propósito General (GPIO)

##### Exportacion de un programa CIAA utilizando Eclipse

[Link al tutorial paso a paso](https://github.com/nachocarballeda/embebidos_fiuba/wiki/TP2:-Tutorial-exportacion-de-programa)

## Migración del proyecto examples/c/app

La migración del proyecto `~/examples/c/app` se realiza al copiar los archivos `app.h` y `app.c` en la carpeta `examples/c/projects/TP2/`, los cuales fueron
renombrados a `TP2.h` y `TP2.c` respectivamente. Esto es necesario ya que el *makefile* requiere que el código fuente del proyecto a construir tenga el mismo nombre que su directorio.

Además, se debe crear los archivos `board.mk` y `program.mk`. Para el primero, se debe especificar la plataforma a utilizar,

```
# board.mk
# Board ------------

BOARD = edu_ciaa_nxp
```

En el archivo `program.mk` debe indicarse la ruta del proyecto y el nombre del directorio,

```
# Program path and name ----------

PROGRAM_PATH = examples/c/projects
PROGRAM_NAME = TP2
```

Por último, para poder utilizar el *debugger*, se debe ir a la configuración del mismo y agregar el archivo de extensión `.elf` en el campo **C/C++ Application**  **Search Project...** de la solapa **Main**.  

![debugger_configuration_1](https://github.com/mollykei/SE_G2/blob/tp_iglesias/TP2/imgs/2_config_debugger.png)

![debugger_configuration_2](https://github.com/mollykei/SE_G2/blob/tp_iglesias/TP2/imgs/3_program_selection.png)

## Secuencia de Funciones

### Diagrama de Secuencia de Funciones

En el siguiente diagrama se presenta la secuencia de funciones invocadas durante la ejecución del código fuente `app.c`.

![function_call_graph](https://github.com/mollykei/SE_G2/blob/tp_iglesias/TP2/imgs/function_calls.png)


### Lista de Funciones Invocadas

A continuación se presenta un listado con las funciones invocados con sus respectivos datos,

#ToDo: agregar los tipos de variables y sus respectivos nombres. Tambien agregar una breve descripcion de cada funcion.

```
main() : int
	boardInit() : void
		SystemCoreClockUpdate() : void
			SystemCoreClock : ?
			Chip_Clock_GetRate(?) : ?
				Chip_Clock_FindBaseClock(?) : ?
					periph_to_base : const CLK_PERIPH_TO_BASE_T [] (4 matches)
					(anonymous)::clkbase : ? (2 matches)
					(anonymous)::clkstart : ?
					(anonymous)::clkend : ?
				Chip_Clock_GetBaseClocktHz(?) : ?
					Chip_Clock_GetClockInputHz(?) : ?
						ExtRateIn : const ?
						OscRateIn : const ?
						audio_usb_pll_freq : ? [] (2 matches)
						Chip_Clock_GetMainPLLHz() : ?
							Chip_Clock_GetClockInputHz(?) : ?
						Chip_Clock_GetDivRate(?, ?) : ? (5 matches)
							Chip_Clock_GetDividerSource(?) : ?
							Chip_Clock_GetDividerDivisor(?) : ?
							Chip_Clock_GetClockInputHz(?) : ?
					Chip_Clock_GetBaseClock(?) : ?
		cyclesCounterInit(?) : bool_t
			ClockSpeed : ?
		SystemCoreClock : ?
		tickInit(tick_t) : bool_t
			tickPowerSet(bool_t) : void (2 matches)
			tickRateMS : tick_t
			SysTick_Config(?) : ?
				SysTick_Config(?) : ?
					(anonymous)::LOAD : volatile ?
					__NVIC_SetPriority(IRQn_Type, ?) : void
						(anonymous)::IP : volatile ? [8] (2 matches)
						(anonymous)::SHP : volatile ? [2] (2 matches)
					SysTick_IRQn
					(anonymous)::VAL : volatile ?
					(anonymous)::CTRL : volatile ?
				SysTick_Config(?) : ?
					(anonymous)::LOAD : volatile ?
					__NVIC_SetPriority(IRQn_Type, ?) : void
						(anonymous)::IP : volatile ? [240]
						(anonymous)::SHP : volatile ? [12]
					SysTick_IRQn
					(anonymous)::VAL : volatile ?
					(anonymous)::CTRL : volatile ?
				SysTick_Config(?) : ?
					(anonymous)::LOAD : volatile ?
					__NVIC_SetPriority(IRQn_Type, ?) : void
						(anonymous)::IP : volatile ? [240]
						(anonymous)::SHP : volatile ? [12]
					SysTick_IRQn
					(anonymous)::VAL : volatile ?
					(anonymous)::CTRL : volatile ?
			SystemCoreClock : ?
	delay(tick_t) : void (2 matches)
		delay(tick_t) : void (2 matches)
			tickRead() : tick_t (2 matches)
				tickCounter : tick_t
			tickRateMS : tick_t
		delay(tick_t) : void (2 matches)
			tick_ct : volatile ? (2 matches)
	gpioRead(gpioMap_t) : bool_t (2 matches)
		gpioObtainPinInit(gpioMap_t, ? *, ? *, ? *, ? *, ? *) : void
			gpioPinsInit : const pinInitGpioLpc4337_t [] (5 matches)
			(anonymous)::pinName : pinInitLpc4337_t (2 matches)
			(anonymous)::port : ?
			(anonymous)::pin : ?
			(anonymous)::func : ?
			(anonymous)::gpio : gpioInitLpc4337_t (2 matches)
			(anonymous)::port : ?
			(anonymous)::pin : ?
		Chip_GPIO_ReadPortBit(LPC_GPIO_T *, ?, ?) : bool
			(anonymous)::B : ? [128][32]
	gpioWrite(gpioMap_t, bool_t) : bool_t
		gpioObtainPinInit(gpioMap_t, ? *, ? *, ? *, ? *, ? *) : void
			gpioPinsInit : const pinInitGpioLpc4337_t [] (5 matches)
			(anonymous)::pinName : pinInitLpc4337_t (2 matches)
			(anonymous)::port : ?
			(anonymous)::pin : ?
			(anonymous)::func : ?
			(anonymous)::gpio : gpioInitLpc4337_t (2 matches)
			(anonymous)::port : ?
			(anonymous)::pin : ?
		Chip_GPIO_SetPinState(LPC_GPIO_T *, ?, ?, bool) : void
			(anonymous)::B : ? [128][32]
	gpioToggle(gpioMap_t) : bool_t
		gpioWrite(gpioMap_t, bool_t) : bool_t
			gpioObtainPinInit(gpioMap_t, ? *, ? *, ? *, ? *, ? *) : void
				gpioPinsInit : const pinInitGpioLpc4337_t [] (5 matches)
				(anonymous)::pinName : pinInitLpc4337_t (2 matches)
				(anonymous)::port : ?
				(anonymous)::pin : ?
				(anonymous)::func : ?
				(anonymous)::gpio : gpioInitLpc4337_t (2 matches)
				(anonymous)::port : ?
				(anonymous)::pin : ?
			Chip_GPIO_SetPinState(LPC_GPIO_T *, ?, ?, bool) : void
				(anonymous)::B : ? [128][32]
		gpioRead(gpioMap_t) : bool_t
			gpioObtainPinInit(gpioMap_t, ? *, ? *, ? *, ? *, ? *) : void
				gpioPinsInit : const pinInitGpioLpc4337_t [] (5 matches)
				(anonymous)::pinName : pinInitLpc4337_t (2 matches)
				(anonymous)::port : ?
				(anonymous)::pin : ?
				(anonymous)::func : ?
				(anonymous)::gpio : gpioInitLpc4337_t (2 matches)
				(anonymous)::port : ?
				(anonymous)::pin : ?
			Chip_GPIO_ReadPortBit(LPC_GPIO_T *, ?, ?) : bool
				(anonymous)::B : ? [128][32]
```

##### Manejos de Entradas y Salidas

# Creación de las funciones de inicializacion, escritura y lectura.


## ToDo: identificacion de las estructuras que respresentan los perifericos SCU y GPIO y las **funciones** implementadas para manejarlas,...

## Configuración del GPIO

En la siguiente tabla se puede observar los tipos de **configuraciones** disponibles para la utilización del GPIO.

| Tipo           | Observación              |
| -------------- |:------------------------:|
| INPUT          | No *PULLUP* o *PULLDOWN* |
| OUTPUT         | -                        |
| INPUT_PULLDOWN       | are neat                 |
| INPUT_REPEATER | *PULLUP* y *PULLDOWN*    |
| INPUT_PULLUP         | -                        |

La información obtenida puede observarse en el tipo enumerativo `gpioInit_t` del archivo *~/libs/sapi/sapi_v0.5.2/board/src/sapi_board.c*.

## Estructura de datos y variables

En primer lugar, se incializan los pines de las entradas/salidas de propósito general disponibles en la *EDU CIAA-NXP*. Para ello, se crea la estructura `_my_gpio_pins_t` que contiene el puerto y pin del **SCU** y **GPIO** que se configurará.

```C
struct _my_gpio_pins_t {
	uint8_t scu_port;
	uint8_t scu_pin;
	uint8_t gpio_port;
	uint8_t gpio_pin;
	uint8_t func;
}; 
```

[Link a la estructura de datos](https://github.com/mollykei/SE_G2/blob/89f43de3445af9ac0b63d856b92cddda22ac6066/TP2/src/my_gpio.c#L9)

Luego, se crean instancias de la estructira `_my_gpio_pins_t` contenidas en un vector. Este contiene la información de los distintos pines y funciones del SCU y GPIO que se utilizan para manejar los leds y pulsadores de la placa _EDU-CIAA_, según la estructura descripta anteriormente. 

```C
const _my_gpio_pins_t gpio_pins_init[] = {
		//{scu_port, scu_pin, gpio_port, gpio_pin, function}
		{ 2, 10, 0, 14, 0 }, //LED1
		{ 2, 11, 1, 11, 0 }, //LED2
		{ 2, 12, 1, 12, 0 }, //LED3
		{ 2, 0, 5, 0, 4 },   //LEDR
		{ 2, 1, 5, 1, 4 },   //LEDG
		{ 2, 2, 5, 2, 4 },   //LEDB
		{ 1, 0, 0, 4, 0 },   //TEC1
		{ 1, 1, 0, 8, 0 },   //TEC2
		{ 1, 2, 0, 9, 0 },   //TEC3
		{ 1, 6, 1, 9, 0 },   //TEC4
		{ 6, 1, 3, 0, 0 },   //GPIO0
		{ 6, 4, 3, 3, 0 },   //GPIO1
};
```

[Link al vector global *_my_gpio_pint_t*](https://github.com/mollykei/SE_G2/blob/89f43de3445af9ac0b63d856b92cddda22ac6066/TP2/src/my_gpio.c#L21)

Finalmente, se crea el tipo enumerativo my_gpio_map_t que contiene todas los posibles *GPIOs*,

```
typedef enum {
	MY_GPIO_LED1,
	MY_GPIO_LED2,
	MY_GPIO_LED3,
	MY_GPIO_LEDR,
	MY_GPIO_LEDG,
	MY_GPIO_LEDB,
	MY_GPIO_TEC1,
	MY_GPIO_TEC2,
	MY_GPIO_TEC3,
	MY_GPIO_TEC4,
	MY_GPIO_GPIO0,
	MY_GPIO_GPIO1,
} my_gpio_map_t;
```

[Link al tipo enumerativo *mu_gpio_map_t*](https://github.com/mollykei/SE_G2/blob/89f43de3445af9ac0b63d856b92cddda22ac6066/TP2/inc/my_gpio.h#L16)


## Funciones

### Inicialización del pin GPIO

En la [función](https://github.com/mollykei/SE_G2/blob/718fcc6d45c7b7f40a5b75d812e2959cc03e9c6e/TP2/src/my_gpio.c#L51) `gpioInit` se puede observar la llamada de las funciones provistas por el fabricante para cada tipo de configuración disponible del *SCU* y *GPIO*,

* `Chip_SCU_PinMux`[SCU], al recibir el número de puerto, el número de pin, el modo de cofiguración y la función del pin, realiza la configuración del pin SCU.

* `Chip_GPIO_SetDir` [GPIO], al recibir el número de puerto, el registro del GPIO, el valor del bit y la dirección,  configura el pin del GPIO como entrada o salida. 

* `Chip_GPIO_SetPinState` [GPIO], al recibir el número de puerto, el registro del GPIO en el integrado, el número del pin y el nivel lógico,  establece  el estado del pin GPIO

### Lectura del pin GPIO

En la [función](https://github.com/mollykei/SE_G2/blob/718fcc6d45c7b7f40a5b75d812e2959cc03e9c6e/TP2/src/my_gpio.c#L111) `gpioRead` se utiliza la funció `Chip_GPIO_ReadPortBit` que lee el estado del pin GPIO al indicar su puerto y pin.

### Escritura del pin GPIO

Al igual que en la [función](https://github.com/mollykei/SE_G2/blob/718fcc6d45c7b7f40a5b75d812e2959cc03e9c6e/TP2/src/my_gpio.c#L127) de inicialización, una vez establecidos los parámetros del tipo enumerativo `my_gpio_config_t`, se llama a la función `Chip_GPIO_SetPinState` detallada previamente para establece el estado del pin GPIO.

---

## Compilación  

Para compilar el programa se deberán seguir los siguientes pasos
1. Ubicar esta carpeta en `firmware_v3/examples/c/tp2`
1. Copiar el archivo `program.mk` y ubicarlo en `firmware_v3` junto con el `Makefile`
1. Ya se podrá compilar el programa normalmente

## app.c
#### Identificar funciones de la librería sAPI
Se identificaron las siguientes funciones de la librería `sAPI` que se utilizan en el programa `app.c`.
- Archivo: `sapi_board.h`
	- `void boardInit(void);`
	> Esta función inicializa los distintos GPIO y el conte de ciclos de clock. 
- Archivo: `sapi_gpio.h`
	- `bool_t gpioRead( gpioMap_t pin );`
	> Lectura de un gpio pin
	- `bool_t gpioWrite( gpioMap_t pin, bool_t value );`
	> Escritura de un gpio pin
	- `bool_t gpioToggle( gpioMap_t pin );`
	> Cambio de estado de un gpio pin


#### Printf
Para redirigir la salida de `printf()` al `UART_USB` se utilizan las siguientes funciones de la librería `lpc_open`.  

```C

//firmware_v3/libs/lpc_open/boards/edu_ciaa_nxp/src/board.c
void __stdio_putchar(int c)
{
   Board_UARTPutChar(c);
}


int __stdio_getchar()
{
   return Board_UARTGetChar();;
}

void __stdio_init()
{
   Board_Debug_Init();
}
```
De esta forma se inicializa el `UART` con una velocidad de `DEBUG_UART_BAUD_RATE 115200`

#### Funcionamiento del programa
Al iniciar el programa se ejecuta la función `boardInit()` que se encarga de inicializar los distintos GPIO. Luego el programa utiliza la función `gpioToggle(CIAA_BOARD_LED)` para prender y apagar un el led de la placa, esto lo realiza por 10 segundos. Luego, entra al ciclo `while(TRUE)` donde prenderá el LED siempre y cuando se presione el botón `CIAA_BOARD_BUTTON`. 
