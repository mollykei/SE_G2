# TP1 Puerta Corrediza

### Descripcion de funcionamiento
La puerta permanecera cerrada en el estado **CERRADO** hasta que se detecte una presencia **evPresencia**. Cuando esto pase el se pasara al estado **ABRIENDO** y al mismo tiempo se hara un raise de la señal interna **siTitilar** la cual encendera y apagara un LED con la idea de que se sepa que se esta ejerciendo accion de control sobre las puertas.

En caso de no detectar presencia **evNoPresencia** la puerta se cerrará completamente pasando al estado **CERRADO**, en caso contrario, si se detecta una presencia se disparara el evento **evPresencia** que comenzara a abrir la puerta nuvamente.

Si el estado **ABRIENDO** dura lo suficiente como para que los motores lleguen a abrir completamente la puerta, los fines de carrera instalados activaran el evento **evAbrio** que llevara al sistema al estado **ABIERTO**.

Una vez en estado **ABIERTO** cuando no se detecte presencia **evNoPresencia** se pasara al estado **CERRANDO** el cual si perdura en el tiempo lo suficiente terminara por cerrar la puerta activando el evento **evCerro** que llevara al sistema otra vez al estado **CERRADO** y desactivara el parpadeo del led con el evento interno **siNoTitilar**.

### Diagrama de estados
![](https://i.imgur.com/vIOP6Uu.png)

### Código Yakindu SCT [puerta_corrediza.sct]
```
/* Control de puerta corrediza */
interface:

// Eventos que son entradas para el sistema
in event evAbrio
in event evNoAbrio
in event evCerro
in event evNoCerro
in event evPresencia
in event evNoPresencia

// Operaciones
operation opMotor(Action:boolean,Status:boolean): void
operation opLuzRoja(Action:boolean,Status:boolean): void

const OPEN: boolean = true
const CLOSE: boolean = false

const ON: boolean = true
const OFF: boolean = false

const ONoFF: boolean = true
const TOGGLE: boolean = true

internal:

//Eventos Internos (señales internas)
event siTitilar
event siNoTitilar
```
