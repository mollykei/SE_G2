# TP1 Horno de Microondas

### Consigna a cumplir
![](https://i.imgur.com/q4nC0j6.jpg)


### Descripcion

#### Eventos que son entradas para el sistema

* **in event evPuertaAbierta**
* **in event evPuertaCerrada**
Eventos que son activados por un sensor que detecta la apertura/cierre de la puerta.

* **in event evSeleccionarModo**
Evento activado por el boton de seleccion de modo. El mismo cambia el modo de funcionamiento del horno.

* **in event evComenzarTerminar**
Evento activado por el boton de Comenzar/Terminar.

* **in event evEncenderLuzInterna**
* **in event evApagarLuzInterna**
Eventos para Encender y Apagar la luz interna del horno.



#### Operaciones
* **opCalentador(Action:boolean, Mode: integer)**
Activa o desactiva el horno en el modo de coccion deseado.

* **opLed(Mode : integer)**
Controla el LED indicador de modo.

* **opLuzInterna(Action: boolean)**
Controla la luz interna del horno.

#### Constantes

Para determinar si el horno esta encendido o apagado:
> const ON: boolean = true
> const OFF: boolean = false
> 

Seleccion modos de coccion:
> const cGRILL:integer = 0
> const cHORNO:integer = 1
> const cHORNOyGRIL:integer = 2

Determinan estado de la puerta:
> const cPUERTAaBIERTA: integer = 0
> const cPUERTAcERRADA: integer = 1

#### Variables utilizadas internamente
> var viModoElegido: integer=1
> var viEstadoPuerta: integer=1
> var viHornoEncendido: boolean=false

#### Eventos Internos (señales internas)
Para mantener actualizado el LED que muestra el modo de coccion seleccionado:

> event siActualizarModo
> 
### Diagramas de estado
#### Diagrama principal del horno
![](https://i.imgur.com/uhNqtHF.png)

#### Diagrama de la puerta
![](https://i.imgur.com/vV55miV.png)

#### Diagrama de la luz interna
![](https://i.imgur.com/eCNPVPs.png)

#### Diagrama del selector de modo
![](https://i.imgur.com/3i6VMme.png)

#### Diagrama del LED indicador de modo
![](https://i.imgur.com/CFLSHB9.png)

