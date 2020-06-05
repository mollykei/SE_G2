## Trabajo práctico 2
#### Sistemas Embebidos
##### Manejos de Entradas y Salidas

# Creación de las funciones de inicializacion, escritura y lectura.


## ToDo: identificacion de las estructuras que respresentan los perifericos SCU y GPIO y las **funciones** implementadas para manejarlas,...

## Configuración del GPIO

En la siguiente tabla se puede observar los tipos de **configuraciones** disponibles para la utilización del GPIO.

| Tipo           | Observación              |
| -------------- |:------------------------:|
| INPUT          | No *PULLUP* o *PULLDOWN* |
| OUTPUT         | -                        |
| PULLDOWN       | are neat                 |
| INPUT_REPEATER | *PULLUP* y *PULLDOWN*    |
| OUTPUT         | -                        |
| ENABLE         | -                        |

La información obtenida puede observarse en el tipo enumerativo `gpioInit_t` del archivo *~/libs/sapi/sapi_v0.5.2/board/src/sapi_board.c*.

## Estructura de datos y variables

En primer lugar, se incializan los pines de las entradas/salidas de propósito general disponibles en la *EDU CIAA-NXP*. Para ello, se crea la estructura `_my_gpio_pins_t` que contiene  miembros de los periféricos **SCU** y **GPIO**.

```
struct _my_gpio_pins_t {
	uint8_t scu_port;
	uint8_t scu_pin;
	uint8_t gpio_port;
	uint8_t gpio_pin;
	uint8_t func;
}; 
```

[Link a la estructura de datos](https://github.com/mollykei/SE_G2/blob/89f43de3445af9ac0b63d856b92cddda22ac6066/TP2/src/my_gpio.c#L9)

Luego, se crea el **vector global** que contiene la inicialización de los pines según la estructura descripta anteriormente. 

```
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

En la [función](https://github.com/mollykei/SE_G2/blob/718fcc6d45c7b7f40a5b75d812e2959cc03e9c6e/TP2/src/my_gpio.c#L51) `gpioInit` se puede observar la llamada de las funciones provistas por el fabricante para cada tipo de configuración disponible del *GPIO*,

* `Chip_SCU_PinMux`[SCU], al recibir el número de puerto, el número de pin, el modo de cofiguración y la función del pin, realiza la configuración del pin GPIO correspondiente a la comunicación UART.

* `Chip_GPIO_SetDir` [GPIO], al recibir el número de puerto, el registrodel GPIO en el integrado, el valor del bit y la dirección,  establece la dirección a utilizar en el puerto del GPIO.

* `Chip_GPIO_SetPinState` [GPIO], al recibir el número de puerto, el registro del GPIO en el integrado, el número del pin y el nivel lógico,  establece  el estado del pin GPIO a través del registro de bytes del GPIO al recibir el número del pin, el nivel lógico, el número y registro del puerto.

### Lectura del pin GPIO

En la [función](https://github.com/mollykei/SE_G2/blob/718fcc6d45c7b7f40a5b75d812e2959cc03e9c6e/TP2/src/my_gpio.c#L111) `gpioRead` se utiliza la funció `Chip_GPIO_ReadPortBit` que lee el estado del pin GPIO al indicar su número, el número y el registro del puerto.

### Escritura del pin GPIO

Al igual que en la [función](https://github.com/mollykei/SE_G2/blob/718fcc6d45c7b7f40a5b75d812e2959cc03e9c6e/TP2/src/my_gpio.c#L127) de inicialización, una vez establecidos los parámetros de la estructura `my_conf_t`, se llama a la función `Chip_GPIO_SetPinState` detallada previamente para establece el estado del pin GPIO.


--------------------------------------------------------------------------------------------------------------------------
Para compilar el programa se deberán seguir los siguientes pasos
1. Ubicar esta carpeta en `firmware_v3/examples/c/tp2`
1. Copiar el archivo `program.mk` y ubicarlo en `firmware_v3` junto con el `Makefile`
1. Ya se podrá compilar el programa normalmente

### app.c
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
