# TP1: Portón Levadizo
## Introducción 

Se quiere realizar el modelo de control de un **portón levadizo** automatizado utilizando máquinas de estado. Para esto se cuentan con los siguientes componentes:

```
1. Motor con movimiento en dos sentidos
2. Sensor de presencia del automovil
3. Fines de carrera
4. Control remoto apertura/cierre
5. Leds indicadores rojo y verde
```

A continuación se muestra el diagrama funcional del portón.

<div style="align: center; text-align:center;">
<img src="http://drive.google.com/uc?export=view&id=1BF9C9t_fUU7K48wcl0_z1Dav8f5bp2n_" />
<div>Imagen realizada por el Ing. Juan Manuel Cruz</div>
</div>

### Eventos y acciones

Se realizó la identificación de los distintos **eventos** y **acciones**  posibles del sistema.

1. Eventos del Sistema
	- `evAbierto`
	- `evCerrado`
	- `evControlRemotoAbrir`
	- `evControlRemotoCerrar`
	-  `evPresencia`
	- `evNoPresencia`
	- `siPrenderLedRojo`
	- `siPrenderLedVerde`
	
> **Nota:** Los nombres de los eventos son los que se utilizaron para programar la máquina en **Yakindu**

Los eventos *Abierto* y *Cerrado* se corresponden con las señales que puede proporcionar el sensor de fin de carrera, esté indicará cuando la puerta del portón esté totalmente cerrada o abierta. Por otro lado, se tienen dos eventos que indican la señal que el control remoto envía, *Abrir* y *Cerrar*. Por último, el sensor de presencia se encargará de generar los eventos *Presencia* y *No Presencia*. Luego de identificar los **eventos** se procedió a identificar las **acciones** que se llevarán a cabo.

2. Acciones Del Sistema
	* ` opMotorGiro(Prender: boolean, Sentido: boolean): void`
	* `opFrenarMotor(void): void`
	* `opLedEstado(Prender: boolean, Led: integer): void`

* La operación `opMotorGiro(Prender: boolean, Sentido: boolean): void` recibe dos variables tipo *boolean* que le indican el sentido de giro y si se deberá prender o apagar el motor. Esta operación no tiene retorno.
* La operación  `opFrencarMotor() `no recibe argumentos, se encarga de frenar el motor y se utiliza en caso de un repentino cambio de giro
* La función `opLedEstado(Prender: boolean, Led: integer): void` se encarga de apagar o prender los distintos leds indicadores que forman parte del sistema. 

### Estados
Luego de identificados los **eventos** y **acciones** se procedió a definir los distintos estados del sistema.

3.  Estados
	- `CERRADO`
	- `ABRIENDO`
	- `CERRANDO`
	- `ABRIENDO`
	- `FRENAR_CERRANDO`
	- `FRENAR_ABRIENDO`
	- `LED_VERDE_PRENDIDO`
	- `LED_ROJO_PRENDIDO`


## Descripción del funcionamiento

El portón permanecerá en el estado `CERRADO` y con el led verde prendido `LED_VERDE_PRENDIDO` hasta que se reciba el evento del control remoto `evControlRemotoAbrir`. Una vez recibido este evento se prenderá el led rojo `LED_ROJO_PRENDIDO` en modo *titlar* y se pasará al estado `ABREINDO`. Al entrar  a este estado el motor pasará de apagado a prendido y se abrirá el **portón**.  En este estado se pueden recibir dos eventos, el primero es el evento del fin de carrera, `evAbierto`, que indicará que portón está abierto y que se debe apagar el motor pasando al estado `ABIERTO`. El otro evento que se puede recibir es `evControlRemotoCerrar`, en caso de que el usuario en el momento de apertura del portón se haya arrepentido y quiera cerrarlo. En ese caso se pasará del evento `ABREINDO` al estado `FRENAR_MOTOR`, donde se frenará el motor para luego invertir su giro en el estado `CERRANDO`.  Además, a este último estado se podrá acceder desde `ABIERTO` una vez que no se detecte más la presencia del vehículo.  En `CERRANDO` el motor se activará girando en el sentido en el que el portón se cierra. Si al estar en este estado se detecta alguna presencia se cancelara el la operación y el portón volverá al estado `ABRIENDO` luego de frenar el motor para cambiar, si dañarlo, su sentido de giro. 

## Máquina De Estado

> La máquina de estado se realizó en el IDE eclipse utilizando el plug-in de **Yakindu**

<div style="align: center; text-align:center;">
<img src="http://drive.google.com/uc?export=view&id=11ClILswSxNa_OhqMNeAazbx0waAgO7Nj" />
<div>Diagrama de estados realizado en Yakindu</div>
</div>

## Código

A continuación se presenta el código de la máquina de estado realizada en **Yakindu**

```
interface Porton:
//Eventos
in event evAbierto
in event evCerrado
in event evControlRemotoAbrir
in event evControlRemotoCerrar
in event evPresencia
in event evNoPresencia
//Acciones
operation opMotorGiro(
	Prender: boolean,
	Sentido: boolean
): void
operation opMotorQuieto() : boolean
operation opFrenarMotor() : void
operation opLedEstado(
	Prender: boolean,
	Color: integer
): void
//Constantes
const ON: boolean = true
const OFF: boolean = false
const ABRIR: boolean = true
const CERRAR: boolean = false
const LED_VERDE: integer  = 0
const LED_ROJO: integer = 1
//Variables

internal:
//Señales internas
event siLedVerde
event siLedRojo
//Tiempo de prendido de los leds
var viMiliSegundos: integer = 500
```
