# TP1 - Panel de Control
## Introducción 

Se realizó el modelo de un panel de Control de un Generador de Señales, de acuerdo a las siguiente especificaciones:

```
- Rango de tensión de salida: 0 a 10V.
- Frecuencia de salida configurable entre 20 Hz y 20 kHz
- Forma de señal triangular, cuadrada y senoidal.
- Selección manual de la tensión de salida, frecuencia y forma de onda.
```

Para implementar el panel de control del generador de señales se supuso que se cuenta con los siguientes pulsadores:

```
- Pulsador Forma (selecciona la forma de señal)

- Pulsador Frecuencia/Tension (selecciona la magnitud a incr./decr.)

- Pulsador Up (incrementa la magnitud)

- Pulsador Down (decrementa la magnitud)
```

## Eventos y acciones

### Eventos
En primer lugar, se realizó la identificación de los **eventos** y  las **acciones** del sistema.

1. Eventos externos del sistema
    + evForma
    + evMagnitud
    + evSubir
    + evBajar
    + evModificarTension
    + evModificarFrecuencia

2. Eventos internos del sistema 
     + evEncenderLed
     + evApagarLed

### Acciones
3. Acciones externas del sistema
      + opSetearForma(FORMnumber:integer): void
      + operation opSetearMagnitud(MAGnumber:integer): void

      + operation opIncrementarFrecuencia(): void
      + operation opDecrementarFrecuencia(): void

     + operation opIncrementarTension(): void
     + operation opDecrementarTension(): void

    + operation opLedTension(Limit: boolean): void
    + operation opLedFrecuencia(Limit: boolean)


> **Nota:** Los nombres de los eventos y las acciones son los que se utilizaron para programar la máquina en **Yakindu**

### Estados
Posteriormente, se identificaron y agruparon los **estados**.

4.  Estados
     + `TRIANGULAR`
     + `CUADRADA`
    + `SENOIDAL`
    + `TENSION`
    + `FRECUENCIA`
    + `OFF`
    + `ON`


## Descripción de la máquina de estado

> La máquina de estado se realizó en el IDE eclipse utilizando el plug-in de **Yakindu**


### Región 1 - Forma Señal

En la primera región, `forma de la señal`, se tienen las tres formas de la señal, `TRIANGULAR`, `CUADRADA` y `SENOIDAL`, creadas como estados de la máquina. Por defecto, el estado es `TRIANGULAR` hasta que el usuario utilice el pulsador destinado a la forma de la señal y en efecto, se produzca el evento `evForma`. Luego, se ejecutará la operación `opSetearForma` según el tipo enumerativo que recibe por argumento. 

El comportamiento de los estados se dará de forma cíclica determinada por el evento `evForma`.



![](http://drive.google.com/uc?export=view&id=1PuqPrto8Iv4MF-F_pza2cozjMWcQvWe0)

### Región 2 - Magnitudes

De manera paralela a la región `Forma Señal` se encuentra el funcionamiento de los pulsador destinado a modificar la tensión o la frecuencia.

Por defecto, el pulsador se encuentra posicionado para modicar la tensión por medio de la operación `opSetearMagnitud` recibiendo por parámetro el tipo enumerativo `TENSION`. El usuario cuenta con un pulsador para elevar o reducir la tensión indicados por los eventos `evSubir` y `evBajar` respectivamente. Además, se ha propuesto un limitador de tensión máxima que fija la tensión en 10 V e informa a las región `LED Tensión` cumpliendo el rol de component `HMI`.  

Al producirse el `evMagnitud`, la conducta de la operación `opSetearMagnitud` y los eventos `evSubir` y `evBajar` en el estado `FRECUENCIA` serán idénticos a los ocurridos en el estado `TENSION`. 

![](http://drive.google.com/uc?export=view&id=1FwTllYdWVDABO6rojdCJezCobq1CF0kR)

### Regiones 3 y 4 - LEDs Frecuencia y Tensión

Las regiones de `LEDs` cuentan con un estado de encendido (`ON`) que se dará al producirse el `evSubir` en el caso que el valor de la tensión/frecuencia, `viTension`/`viFrecuencia` sea máximo y el de apagado (`OFF`) cuando `viTension`/`viFrecuencia` sea menor al máximo valor. 

![](http://drive.google.com/uc?export=view&id=1lUQomCcSXjfhNjPxp0p6_KOvZvxX6sDf)

![](http://drive.google.com/uc?export=view&id=1JzH3k5qGGMl7WG5bf_ubxNb_fl_tD8Oo)


## Código

A continuación se presenta el código de la máquina de estado realizada en **Yakindu**.

```
interface:

in event evForma
in event evMagnitud
in event evSubir
in event evBajar

in event evModificarTension
in event evModificarFrecuencia


operation opSetearForma(FORMnumber:integer): void
operation opSetearMagnitud(MAGnumber:integer): void

operation opIncrementarFrecuencia(): void
operation opDecrementarFrecuencia(): void

operation opIncrementarTension(): void
operation opDecrementarTension(): void

operation opLedTension(Limit: boolean): void
operation opLedFrecuencia(Limit: boolean)

const ON: boolean = true
const OFF: boolean = false

// constantes Forma
const TRIANGULAR: integer = 0
const CUADRADA: integer = 1
const SENOIDAL: integer = 2

// constantes Magnitud
const TENSION: integer = 0
const FRECUENCIA: integer = 1

internal:

var viTension: integer = 0
var viFrecuencia integer = 0

in event evEncenderLed
in event evApagarLed
```
