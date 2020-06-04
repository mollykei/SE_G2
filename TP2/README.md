# Trabajo Práctico nro. 2 - LPC43xx Entradas y Salidas (Digitales) de Propósito General (GPIO)

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

Por último, para poder utilizar el *debugger*, se debe ir a la configuración del mismo y agregar en el campo **C/C++ Application**  **Search Project...** de la solapa **Main** el archivo de extensión `.elf`.  

![debugger_configuration_1](https://github.com/mollykei/SE_G2/blob/tp_iglesias/TP2/imgs/2_config_debugger.png)

![debugger_configuration_2](https://github.com/mollykei/SE_G2/blob/tp_iglesias/TP2/imgs/3_program_selection.png)

## Secuencia de Funciones

### Diagrama de Secuencia de Funciones

En el siguiente diagrama se presenta la secuencia de funciones invocadas durante la ejecución del código fuente `app.c`.

![function_call_graph](https://github.com/mollykei/SE_G2/blob/tp_iglesias/TP2/imgs/function_calls.png)


### Lista de Funciones Invocadas

A continuación se presenta un listado con las funciones invocados con sus respectivos datos,

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




