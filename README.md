# Experimental-VTOL
Firmware experimental para controlador de vuelo basado en Atmega2650 y MEMS MPU6050 de configuración "bicopter". 

Diseñado para obtener datos de vuelo de las fases de despegue y aterrizaje vertical mediante el uso de dos motores brushless y 2 servo motores, parte del material de consulta para proyectos de nivel escolar del Instituto Politécnico Nacional Campus Guanajuato, como propuesta de grupo para la distribución de software libre con fines educativos.

El firmware presentado en este proyecto es unicamente con fines experimentales y educativos para los ingenieros aeronáuticos en formación y entusiastas, ya que no represente DE NINGUNA MANERA un firmware adecuado para etapa de producción y el autor no se hará responsable de los infortunios ocasionados por el mal uso del mismo.

El código presenta las siguientes etapas:
- Obtención de información de utilidad del sensor MPU6050.
- Lectura de las señales de radio controles usuales en el aeromodelismo, mediante el uso de rutinas de interrupción.
- Procesamiento de los datos del sensor para poder hacerlos usables.
- Rutinas de seguridad "anti - armado" para evitar accidentes con hélices de hobby.
- Failsafe con funcionamiento unico para protocolos de productos FrSky (sin embargo, se incluye una propuesta para hacer uso de la función failsafe para productos Turnigy/FlySky).
- Procesamiento de los valores de salida mediante el uso de un controlador PID y la teoria de control clásica.
- Creación de pulsos PWM para ESC's de protocolos convencionales (NO ONE-SHOT125, MULTISHOT O DSHOT).
- Documentación apropiada para el nivel medio de carrera de Ingenieria en Aeronáutica.



La propuesta forma parte del proyecto de investigación para la titulación y conforma una serie de materiales de ayuda para permitir a los ingenieros en formación tener un punto de partida para poder desarrollar los proyectos de último semestre, en especial relacionados con las materias de:

- Sistemas de control en aeronaves
- Construcciones aeronátucias
- Dinámica del vuelo

Entre otras.

Por último, recordar que se libera bajo licencia GPLv3, por lo que cualquier derivación de este proyecto debe ser liberada bajo lo establecido por dicha licencia. Espero que los estudiantes encuentren el proyecto agradable y sobre todo UTIL.

