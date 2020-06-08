#  TP1 Escalera Mecánica

  

###  Consigna a cumplir

Escalera Mecánica unidireccional automatizada. Con un motor con movimiento en un sentido y dos velocidades, sensores de ingreso, egreso y señalización luminosa.

  

###  Descripcion de funcionamiento

  

La escalera mecánica permanece en modo apagado **ESCALERA APAGADA** hasta que el un usuario la encienda presionando el botón de encender (dispara **evEncenderEscalera**) lo cual lleva al estado **ESCALERA FUNCIONANDO**. Como dato para entender el funcionamiento de la misa, se considera que la escalera funciona con dirección hacia arriba.

Mientras la escalera está apagada, el usuario puede seleccionar una de dos velocidades, en ese momento la escalera permanece apagada, en el estado **CAMBIANDO VELOCIDAD**.

El sistema de señalización luminosa funciona cuando la escalera esta encendida, es decir, cuando la escalera está apagada, la luz se encuentra en el estado **LUZ APAGADA**.

Cuando la escalera se enciende, en velocidad 1, la luz pasa al estado **LUZ TITILA LENTO** y cuando la escalera se enciende en velocidad 2, la luz pasa al estado **LUZ TITILA RÁPIDO**

Finalmente la escalera tiene un sensor de ingreso y egreso de personas. Funciona tanto con la escalera apagada como con la escalera encendida.

Cuando una persona sube en la "Planta Baja" se contabiliza que hay una persona usando la escalera, cuando una persona llega a la "Planta alta", se contabiliza que una persona sale de la escalera.

Mientras no haya personas en la escalera, el sensor se mantiene en el estado **ESCALERA SIN GENTE**

  

###  Diagrama de estados

![Diagrama de escalera mecánica](https://i.imgur.com/pxcDT6R.png)
![Diagrama de señalizaciones luminosas](https://i.imgur.com/NO5P9Jm.png)
![Diagrama de contro de ingreso y egreso](https://i.imgur.com/Y4RsTf4.png)

  

  

###  Código Yakindu SCT [escalera_mecanica.sct]

```

/* Control escalera mecanica */

/** Consigna:

* (Unidireccional

* 2 velocidades

* sensor de ingreso/egreso

* señalizaciones luminosas)

*/

//todo lo que se relaciona con el mundo exterior

interface:

// Eventos que son entradas para el sistema

in event evApagarEscalera

in event evEncenderEscalera

in event evCambiarVelocidad

in event evSubeGente

in event evBajaGente

// Operaciones

operation opMotor(Action:boolean, Velocity: integer): void

operation opLuz(Action:boolean):void

const ON: boolean = true

const OFF: boolean = false

const LUZ_ON:boolean = true

const LUZ_OFF:boolean = false

internal:

var viVelocidad: integer=1

var viPersonas:integer=0

//Eventos Internos (señales internas)

event siTitilar

event siApagarLuz

```
