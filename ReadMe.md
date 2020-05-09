## Sistemas Embebidos - Grupo 2

### Integrantes

 - *Fernando Iglesias*
 - *Ignacio L. J. Carballeda* [\<Repositorio personal de la materia\>](https://github.com/nachocarballeda/embebidos_fiuba)
 - *Katrine Poulsen*
 - *Matías Sambrizzi*

### Tutorial StateCharts Yakindu

#### Instalacion

##### Instalando CIA Launcher

El primer paso es instalar la aplicacion llamada **CIA Launcher**, para ello recomendamos fuertemente leer la documentación detallada sobre el _Paquete de herramientas listas para usar para la programación de las plataformas del Proyecto CIAA_ ubicado en https://github.com/ciaa/software/.

Una vez instalado, se verá asi:

![alt text](https://github.com/ciaa/software/blob/master/applauncher/docs/launcher-linux.png)

##### Instalando Plugins para Eclipse

Dentro de Eclipse, abrir el Marketplace para buscar e instalar los siguientes plugins:

 - egit (Github Integration for Eclipse 5.6.1)
 - Yakindu (Statechart Tools 3.5.10)
 
![alt text](https://user-images.githubusercontent.com/9441622/81479259-9242f500-91f8-11ea-9956-31f5540f0ec8.png) 

##### Realizando pedido de licencia Yakindu

Entrar a la web de [Yakindu](https://www.itemis.com/en/yakindu/state-machine/licenses) y realizar el pedido de una licencia profesional para estudiantes universitarios utilizando la cuenta de email de fiuba (@fi.uba.ar).

![alt text](https://user-images.githubusercontent.com/9441622/81479258-90793180-91f8-11ea-9daa-58f640441147.png)

Seguir los pasos. Confirmar el email con los terminos/condiciones y luego esperar (no deberia tomar mas que 4 dias habiles).

##### Añadiendo licencia Yakindu

Dentro de Eclipse->windows->Preferences Entrar a Yakindu Licenses y añadir la que nos enviaron por email.
Nos deberia quedar asi:
![alt text](https://user-images.githubusercontent.com/9441622/81479429-72f89780-91f9-11ea-861d-5faf60e0cc63.png)

##### Probando que Yakindu funciona correctamente

Dentro de Ecplipse, en la pestaña de **Project_Explorer** deberia figurarnos la carpeta con el firmware_v3 de la CIAA. Para probar el funcionamiento de _Yakindu_ podemos hacer lo siguiente. Entrar a examples->C->sapi->statecharts->1_toggle , luego hacer doble click en el archivo **toggle.sct**, se deberia ver la maquina de estados correspondiente al ejemplo _Toggle_.

Hacer el click derecho en el archivo toggle.sct->Run As->Statechart Simulation. Esto deberia correr la simulacion, que se vera de la siguiente manera.

![alt text](https://user-images.githubusercontent.com/9441622/81479641-c3bcc000-91fa-11ea-9150-03767ebbc243.png)
