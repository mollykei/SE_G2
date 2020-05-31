## Trabajo práctico 2
#### Sistemas Embebidos
##### Manejos de Entradas y Salidas

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
