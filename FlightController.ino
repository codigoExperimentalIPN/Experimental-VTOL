/*
El siguiente código es parte del proyecto de titulación "Diseño de un controlador de vuelo para una aeronave no tripulada con capacidades de despegue y aterrizaje vertical".
Septiembre 2017
Escrito por : Yannick Tonatiuh Napsuciale Heredia

El código se considera en una fase experimental y no se recomienda para fase de producción, asi mismo se encuentra liberado bajo la licencia MIT o X11, la cual establece lo siguiente:

Copyright (c) <2017> 

Se concede permiso, de forma gratuita, a cualquier persona que obtenga una copia de este software y de los archivos de documentación asociados (el "Software"),
para utilizar el Software sin restricción, incluyendo sin limitación los derechos a usar, copiar, modificar, fusionar, publicar, distribuir, sublicenciar, 
y/o vender copias del Software, y a permitir a las personas a las que se les proporcione el Software a hacer lo mismo, sujeto a las siguientes condiciones:
El aviso de copyright anterior y este aviso de permiso se incluirán en todas las copias o partes sustanciales del Software.

EL SOFTWARE SE PROPORCIONA "TAL CUAL", SIN GARANTÍA DE NINGÚN TIPO, EXPRESA O IMPLÍCITA, INCLUYENDO PERO NO LIMITADO A GARANTÍAS DE COMERCIALIZACIÓN, 
IDONEIDAD PARA UN PROPÓSITO PARTICULAR Y NO INFRACCIÓN. EN NINGÚN CASO LOS AUTORES O TITULARES DEL COPYRIGHT SERÁN RESPONSABLES DE NINGUNA RECLAMACIÓN, DAÑOS U OTRAS RESPONSABILIDADES, 
YA SEA EN UNA ACCIÓN DE CONTRATO, AGRAVIO O CUALQUIER OTRO MOTIVO, QUE SURJA DE O EN CONEXIÓN CON EL SOFTWARE O EL USO U OTRO TIPO DE ACCIONES EN EL SOFTWARE.

Aclarando el punto anterior, el autor expresa su deseo de encontrar de utilidad para el desarrollo proyectos, sean escolares o no, pero relacionados con la aeronáutica.
A su vez se indica que el código funciona unicamente con el hardware mencionado en el documento del cual este código es anexo.

*/

/* El siguiente código se libera bajo una licencia con filosofia de código de abierto, sin embargo, el autor no se hace responsable de daños de cualquier tipo
asi mismo, el autor no recomienda usar este código con fines de producción ya que se encuentra en una etapa completamente experimental.
El firmware funciona con un hardware especifico y conectado como se indica en la documentación del proyecto.

NOTA DEL AUTOR:
El siguiente código se escribe con el fin de iniciar el interes de los alumnos de la licenciatura en Ingenieria en Aeronautica en la escritura de firmaware
/software para fines de cualquier sector de la aeronautica, por ejemplo, sistemas no tripulados y autopilotos.

Se pretende documentar el código para que quede lo mas entendible posible para las personas que posean poco a nulo conocimiento de la plataforma arduino y/o 
el lenguaje de programación C++ y las librerias propias del entorno de desarollo de Arduino, que se define como una plataforma introductoria para 
el prototipado rapido.

El firmware pretende cumplir con el objetivo de desempeñar el despegue vertical de una aeronave hibrida con dicha capacidad.

Sin mas, el autor espera que los lectores de la documentación del proyecto encuentren su contenido entendible y despierte su interes en el desarrollo del mismo proyecto,
pero implementado mejoras y caracteristicas al mismo. 	

El controlador en cuestión cubre la comunicación con el radio control, las entradas y salidas del sistema
la comunicación con los sensores, la telemetria, el failsafe, el controlador PID que regula la estabilzación/nivelado del prototipo durante la fase
de despegue vertical, el prototipo no posee una modalidad manual/no estabilizada por cuestiones de seguridad, sin embargo, teniendo el nivelado es sumamente sencillo
añadirle dicha funcionalidad
*/

// Librerias a usar, Wire para la comunicación i2c con el sensor acelerometro giroscopio, SPI para la comunicacion con el sensor de radio frecuencia
// nrf24l01 que desempeña la telemetria de los datos de interes del proyecto, y por ultimo la libreria RF24 que contiene las clases para iniciar la comunicación
// con los modulos de telemetria y el envio de datos


#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <Kalman.h>

// Se definen constantes para las direcciones del sensor acelerometro/giroscopio y algunas constantes para la conversion de unidades de datos de medicion

#define direccionSensor 		0x68
#define direccionFiltroPasaBajo 26
#define sensibilidadAcc 		28
#define sensibilidadGyro 		27
#define inicioDeDatos 			59
#define powerManagement 		107
#define radianesAGrados			57.3
#define g 						9.81
#define gradosARadianes			0.01745


// VARIABLES PARA LA COMUNICACIÓN CON EL RADIO CONTROL

int receiver1, receiver2, receiver3, receiver4, receiverAux, receiverVTOL;
byte ultimoCanal1, ultimoCanal2, ultimoCanal3, ultimoCanal4, ultimoCanalAux, ultimoCanalVTOL;
unsigned long timer1, timer2, timer3, timer4, timerAux, timerVTOL;

/*NOTA IMPORTANTE
Dentro del aeromodelismo existen multiples opciones cuando se trata de la adquisicion de un radio control para aeromodelos de ala fija 
tanto como multo rotores, cada uno posee su propia propiedad intelectual en cuanto a la modulacion de las señales y protocolos de comunicacion
Dentro del mercado actual, los radio controles FM ya no son tan comunes por tener una latencia considerable (para el proposito) y el gran tamaño de las antenas,
dicho esto, son comunes los radio controles de 2.4Ghz, y este posee a su vez, protocolos de comunicaciòn analogicos y digitales, siendo los analogicos mas comunes
la modulacion por ancho de pulso o PWM y la modulacion por posicion de pulso o PPM, el primero requiere un cable por canal del radio control y sus respectivos cables
de voltaje y tierra, el protocolo PPM, requiere solo un cable para hasta 8 canales, uno de voltaje y uno de tierra.
Los protocolos digitales comunmente conocidos son DSM2, DSMX, SBUS y IBUS, todos son protoclos digitales de comunicacion serial, existen otros, sin embargo no son tan populares dentro del aeromodelismo.

DSM2 y DSMX son protocolos (siendo el segundo una version mejorada del primero) desarrollados por Spektrum, una compañia dedicada al radio control de alto nivel, y marca
bastante popular dentro de los aeromodelistas de ala fija.

SBUS, desarrollado por Futaba (principal competidor de spektrum en ala fija), que se ha ido expandiendo a otras compañias (al igual que DSMX), debido a que terceros
han logrado decodificar el protocolo usando metodos de ingenieria inversa (lo cual en la mayoria de los paises es completamente legal, y a discrecion de la empresa desarrolladora)
adoptandolo una gran cantidad de empresas como FrSky, Radiolink, entre otras.

IBUS desarrollado por la empresa de cede asiatica, Flysky es una modificacion de SBUS para implementar caracteristicas especificas de los radio controles que desarrollan

Al comparar PWM y PPM, la principal ventaja de usar PPM sobre PWM (ademas de usar un solo cable para hasta 8 canales) es la reduccion del tiempo de latencia entre el transmisor (radio) y
su respectivo receptor, a pesar de estar ambas en al rango de los milisegundos, es bastante perceptible la diferencia ya en uso, teniendo en algunos casos una latencia de hasta 20 ms para PWM
y de hasta 4ms para PPM, lo cual en maniobras agresivas o condiciones precarias de vuelo puede ser la diferencia entre reacciones ante un obstaculo o perder el control de la aeronave
Esto es bastante mas critico en multirotores, sobre todo en los que poseen caracteristicas de vuelo en primera persona.

Los protocolos seriales, ademas de bajar dicha latencia aun mas, pueden almacenar hasta 16 canales en un solo cable, esto es bastante util en multirotores
Sin embargo, la mayoria de los receptores de SBUS, suelen solo soportar SBUS por lo que suelen poseer solo 3 pines, lo cual es inadecuado para el ala fija al necesitar mas pines para conectar los servos
, dicho esto, existen receptores que trabajan con multiples protocolos para ser multiproposito, pero modelos de bajo factor de forma y peso, suelen solo poseer conexiones para SBUS, y se necesita 
un decodificador de servos para SBUS, que viene a ser un modulo externo de medio costo.



IMPORTANTE

ESTE PROYECTO USA EL RADIO TURNIGY QUE SOLO POSEE LA CAPACIDAD DE COMUNIACION POR PWM EN 2.4GHZ, DE IGUAL MANERA, EL FIRMWARE SOLO FUNCIONARÀ BAJO LA CORRECTA CONEXION DE LOS COMPONENTES UNICAMENTE 
COMO SE INDICA EN EL APARTADO "HARDWARE" DEL DOCUMENTO

*/


// VARIABLES PARA EL CONTROLADOR PID

float rollLevel, pitchLevel;
float errorRoll, errorPitch, errorYaw;
float rollIntegralError, rollSetpoint, rollOutput, rollUltimoError;
float pitchIntegralError, pitchSetpoint, pitchOutput, pitchUltimoError;
float yawIntegralError, yawSetpoint, yawOutput, yawUltimoError;
int rollKi = 0.005f, rollKp = 2.0f, rollKd = 10;
int pitchKi = 0.005f, pitchKp = 2.0f, pitchKd = 10;
int yawKi = 0.005f, yawKp = 2.0f, yawKd = 10;
int rollPIDMax = 400, pitchPIDMax = 400, yawPIDMax = 400;


// VARIABLES PARA LAS SALIDAS DEL PROTOTIPO

int throttle, escLeft, escRight, servoLeft, servoRight;
int direccionYaw = 1;
int start, loopTimer, escLoopTimer;
int timerCh1, timerCh2, timerCh3, timerCh4;
int levelForce = 15;


// VARIABLES SENSOR

float accX = 0, accY = 0, accZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0, temp = 0;
float offsetAccX = 0, offsetAccY = 0, offsetAccZ = 0, offsetGyroX = 0, offsetGyroY = 0, offsetGyroZ = 0;
float tempEscalada, accXCalibrado, accYCalibrado, accZCalibrado, gyroXCalibrado, gyroYCalibrado, gyroZCalibrado;
float anguloGyroX = 0, anguloGyroY = 0, anguloGyroZ = 0, anguloAccX = 0, anguloAccY = 0, anguloAccZ = 0, Roll = 0, Pitch = 0, Yaw = 0;
float accVector;

// VARIABLES TELEMETRIA


// Esta constante se usa para codificar la comunicaciòn a un canal especifico, esto con el fin de reducir interferencias
const uint64_t canalTelemetria = 0XE8E8F0F0E1LL; 

RF24 tel(9, 10);				// Definimos el objeto tel (de telemetria), como se indica en la documentación de la libreria RF24

typedef struct {

	float telRoll;
	float telPitch;
	int radioRoll;
	int radioPitch;
	int radioYaw;
	int radioThrottle;
	int VTOLModeActivation;
}	TELEMETRIA;

TELEMETRIA telemetria;


/* Nota importante: el modulo nrf24l01 usando la libreria RF24, solo admite la transmision de variables byte y enteros, no admite variables flotantes
, dicho esto, se codifican las variables de interes en una variable tipo struct, la cual es admitida por el modulo RF y se separan
sus elementos al recibirlo en el modulo que actua como receptor
*/

//  VARIABLES FAILSAFE
// Debido a la forma en la que funcionan los radio controles de 2.4Ghz, si se pierde la comunicación el motor electrico/servo se 
// quedan con la ultima señal recibida por el radio control, esto puede ocasionar que la aeronave continue volando y se pierda.
// Por esta razón se implementa una rutina que de perderse la comunicación con el radio control, apague el motor y el servo quede
// posición neutra

int lastReceiver3Failsafe, errorFailsafe, failsafeFlag;
long contadorFailsafe;
int failsafeMotorPulse = 1000;
int failsafeServoPulse = 1500;

/******************************************* VARIABLES SERVO ************************************************************/

Servo servoDer, servoIzq, servoRoll, servoPitch, servoYaw;



void setup(){


	Serial.begin(38400);								// Iniciamos la comunicacion serial con el microcontrolador
	DDRE |= B00110000;									// Declaramos los pines 3, 4, 5 y 6 del registro H como salidas (mas detalles en el documento)
	DDRB |= B10000000;									// Declaramos el pin 7 del registro B como salida (el LED de Arduino)
	digitalWrite(13, HIGH);								// Prendemos dicho LED
	delay(2000);										// Esperamos 2 seg

	Wire.begin();										// Comando para iniciar la comuniaciòn i2c
	Wire.beginTransmission(direccionSensor);			// Buscamos la comuniaciòn con la direccion del sensor declarado en variables globales
	Wire.write(powerManagement);						// Buscamos el registro de energia del sensor y apagamos el sensor (mas detllaes en el documento)
	Wire.write(0b10000000);
	Wire.endTransmission();								// Terminamos la comunicaciòn y esperamos 100ms
	delay(100);

	Wire.beginTransmission(direccionSensor);			// Se reinicia el sensor usando la misma direccion que los comando anteriores
	Wire.write(powerManagement);						// Esto se hace para tener una calibracion del sensor adecuada
	Wire.write(0b00000000);
	Wire.endTransmission();

	sensibilidadSensores(1, 2);							// Estas 5 clases estan definidas mas abajo, esta linea es para declarar unidades de medicion
	filtroPasabajo(0);									// Elegimos la fuerza del filtro pasabajo para evitar que las vibraciones de los motores afecten las mediciones
	calibracion();										// Rutina de calibraciòn para obtener el desfase de mediciones y corregirlas
	definirInterrupciones();							// Sub rutina para la comunicacion con el radio control



/* Dado que las posiciones del radio control son señales que van de los 1000 a los 2000 us, la siguiente rutina no permite que los motores giren si la palanca
de potencia no esta en la posicion minima y las demas en su posicion central, debido a que los variadores de velocidad o ESC comienzan a emitir un sonido agudo 
al no detectar una señal, mientras no se cumplan las condiciones de arranque del programa mencionadas, mandaremos una señal de 1000 us a los variadores 
(usualmente los motores comienzan a girar en señales cercanas a los 1100, aunque esto varia dependiendo del ESC), sumado a esto, iniciamos un contador para mandar
un parpadeo LED para indicar que no se estan cumpliendo las condiciones de arranque
*/
	while(receiver3 < 900 || receiver3 > 1100 || receiver1 < 1400){
		start++;
		PORTE |= B01100000;
		delayMicroseconds(1000);
		PORTE &= B11001111;
		delay(3);
			if(start == 125){
				digitalWrite(13, !digitalRead(13));
				start = 0;
			}
	}

	servoDer.attach(6);
	servoIzq.attach(7);						// indicar que los servos van conectados en las salidas 6 y 7 de la placa

// Si se cumplen las condiciones de arranque, reiniciamos el contador start y apagamos el LED del PIN 13 del puerto B
	
	start = 0;
	digitalWrite(13, LOW);

/* Segun la documentaciòn de la libreria RF24, debemos iniciar la comunicaciòn del objeto definido en las variables, la libreria tambien 
permite indicar la potencia de salida, la tasa de transferencia de datos, entre otros, en este firmware, solo modificaremos la potencia de
 salida y la tasa de datos, lo primero debe hacerse para evitar sobrecargar el voltaje de salida que puede proveer el arduino para sensores
el segundo podemos dejarlo en el maximo, aunque la utilidad es poca ya que solo requerimos pocos datos en telemetria y la tasa minima
ya provee un muy buena lectura de datos. */

	tel.begin();
	tel.setPALevel(RF24_PA_MIN);				// Potencia de salida minima
	tel.setChannel(115);						// Definir el canal de comunicación
	tel.setDataRate(RF24_250KBPS);				// Min = 250 KBPS , MAX = 2MBPS
	tel.openWritingPipe(canalTelemetria);		// Abrimos el canal de comunicaciòn en la direccion indicada en las variables

	delay(100);
}

void loop(){
	

/* RUTINA DE FAILSAFE 
Al usar el codigo provisto para la comuniacion via PWM con el radio control se puede notar que hay un pequeño retraso ocasional de 4us en las lecturas,
esto nos da la herramienta para diseñar una rutina para implementar el "failsafe" de manera sencilla, y sin necesidad de meterse con los registros del 
microcontrolador que se encargan de ello.

Utilizando el monitor serial se puede observar que si se apaga el radio (es decir se pierde la conexion), dicho retraso deja de existir y los ESC se
quedan con la ultima señal enviada por el transmisor, mencionandolo de nuevo, esto puede ocasionar la perdida del modelo, al tener la capacidad de seguir volando.
Por ello, se implementó un contador que lleve la cuenta cada cierto numero de mediciones, donde deberia existir una diferencia entre la medicion actual y la
anterior debido al retraso mencionado, si despues de 30 mediciones, dicha diferencia no existe, implica que el pulso se ha mantenido en su lugar, y como mencionamos, 
a su vez indica que se ha perdido la comunicaciòn con el radio control, por lo que ante dicho caso, mandaremos una señal a los motores para apagarse, y a los servos
para adoptar la posicion media (o defleccionado en su totalidad para obtener la caida inmediata del modelo, se puede cambiar).
*/

	erroFailsafe = receiver3 - lastReceiver3Failsafe;

		if( errorFailsafe == 0){
			contadorFailsafe++;
		}
		else if( errorFailsafe != 0){
			contadorFailsafe = 0;
			failsafeFlag = 0;
		}

		if( contadorFailsafe > 30){
			escLeft = failsafeMotorPulse;
			escRight = failsafeMotorPulse;
			servoLeft = failsafeServoPulse;
			servoRight = failsafeServoPulse;
		}

		lastReceiver3Failsafe = receiver3;


	
	leerDatos();				// Clase usada para obtener los datos del acelerometro/giroscopio, esta documentada mas abajo

	accVector = sqrt((accX * accX) + (accY * accY) + (accZ * accZ));
	anguloAccY = asin((float) accY / accVector) * radianesAGrados;
	anguloAccX = asin((float) accX / accVector) * radianesAGrados;

	anguloGyroX += (gyroX / 16375);		// la constante 16375 es el valor indicado por el manual del sensor para la conversion a grados
	anguloGyroY += (gyroY / 16375);		// en una sensibilidad de 500dps
	anguloGyroZ += (gyroZ / 16375);


/* Durante la comprobación del funcionamiento del controlador de vuelo se notó que el la revision 4 del firmware, daba lecturas de posicion
angular incorrecta bajo un caso especifico, al efectuar un movimiento de guiñada habiendo ya una inclinación en cualquier de los otros dos
ejes, por lo que al tomar mediciones constantes se notó existia una transferencia entre los tres movimientos relacionada con la guiñada, 
dicho efecto puede corregirse añadiendo el sin de la posicion angular en el eje Z a las mediciones de los otros dos ejes.
*/
	anguloGyroX += anguloGyroX * sin((gyroZ / 16375) * gradosARadianes);
	anguloGyroY -= anguloGyroY * sin((gyroZ / 16375) * gradosARadianes);

	
// La siguiente salida de datos es la posición angular final, usa una fusión de sensores (acelerómetro + giroscópio) por medio de un 
// filtro complementario, mas información del filtro complementario en el documento del proyecto	
	Roll = (0.02 * anguloGyroX) + (0.98 * anguloAccX);
	Pitch = (0.02 * anguloGyroY) + (0.98 * anguloAccY);

	
// Estas dos lineas de código expresan la función de auto nivelado del prototipo
	rollLevel = Roll * levelForce;
	pitchLevel = Pitch * levelForce;

// Procedimiento de seguridad para arranque de los motores	
	/*
	Dependiendo de la combinación de switches ejecuta una rutina diferente,
	una de seguridad para evitar que la aeronave entre en modo VTOL mientras aun 
	no ha despegado.
	Una que solo permite el armado de los motores cuando el throttle se encuentra en la posición mínima
	Y dos que permiten la transición entre modo VTOL y modo avión
	*/

  if(start == 0 && receiverAux > 1900){
    start = 1;
  }
  if(start == 0 && receiverAux > 1900 && receiverVTOL > 1900){
    start = 1;
  }
  if(start == 1 && receiverAux < 1100){
    start = 0;
  }
  if(start == 2 && receiverAux < 1100){
    start = 0;
    escRight = 1000;
    escLeft = 1000;
    servoRight = 1500;
    servoLeft = 1500;
    rudderServo = 1500;
    elevatorServo = 1500;
  }
  if(start == 1 && receiverVTOL < 1100 && receiver3 < 1100){
    start = 2;
    errorRoll = 0;
    errorPitch = 0;
    errorYaw = 0;
    rollIntegralError = 0;
    pitchIntegralError = 0;
    yawIntegralError = 0;
    posVTOL = 1000;
  }


	rollSetpoint = 0;				// Estas tres variables definen el punto sobre el cual el controlador PID corregirá la posición angular
	pitchSetpoint = 0;				// siempre y cuando el nivel se desee en 0 (palancas centradas)
	yawSetpoint = 0;



/*
La siguiente parte del firmware le indica al controlador cuando el setpoint ha cambiado, esto se da cuando se desea hacer un movimiento de roll y pitch
en donde el setpoint depende directamente de la posición de las palancas de dichos movimientos (siempre y cuando no se encuentre en su posición central)
Los múltiples condiciones son debido a que hay que tomar en cuenta el retraso de 4us en la señal, por lo que declaramos una sección muerta dentro del rango
del radio control en donde el setpoint seguirá siendo 0, por seguridad.
*/
	if( receiver1 > 1510 ){
		rollSetpoint = receiver1 - 1510;
	}
	else if( receiver1 < 1490 ){
		rollSetpoint = receiver1 - 1490;
	}
	rollSetpoint -= rollLevel;

	if( receiver2 > 1510 ){
		pitchSetpoint = receiver2 - 1510;
	}
	else ( receiver2 < 1490 ){
		pitchSetpoint = receiver2 - 1490;
	}
	pitchSetpoint -= pitchLevel;

	if( receiver4 > 1510 ){
		yawSetpoint = receiver4 - 1510;
	}
	else if( receiver4 < 1490 ){
		yawSetpoint = receiver4 - 1490;
	}


// Clase usada para los calculos del controlador PID, este código se encuentra mas abajo
	calcularPID();

//----------------------------------------------------------- SALIDAS ----------------------------------------------------------------------

	throttle = receiver3;
	if( start == 2 ){
		if(throttle > 1800 ){
			throttle = 1800;
		}

		escLeft = throttle - rollOutput;
		escRight = throttle + rollOutput;
		servoLeft = constrain( 1500 + direccionYaw * (yawOutput - pitchOutput), 1000, 2000);
		servoRight = constrain( 1500 + direccionYaw * (yawOutput + pitchOutput), 1000, 2000);

		if(escLeft < 1150){
          escLeft = 1150;				// evitar que los motores se apaguen al 
        }								// bajar la palanca de throttle
        if(escLeft > 2000){
          escLeft = 2000;
        }
        if(escRight < 1150){
          escRight = 1150;
        }
        if(escRight > 2000){
          escRight = 2000;
        }
        if(servoRight > 2000){
          servoRight = 2000;
        }
        if(servoRight < 1000){
          servoRight = 1000;
        }
        if(servoLeft > 2000){
          servoLeft = 2000;
        }
        if(servoLeft < 1000){
          servoLeft = 1000;
        }
	}
	else {
		escLeft = 1000;
		escRight = 1000;
		servoLeft = 1500;
		servoRight = 1500;
	}

/* El código escrito a partir del comentario "salidas" se usa para darle forma a la mezcla de los motores, dicha mezcla depende de la cantidad de motores y
su distribución, sin embargo, haciendo una animación mental de los movimientos que queremos que efectue nuestra aeronave, es bastante intuitivo hacer mezclas propias

En este caso el movimiento de Roll esta controlado directamente por las rpms de ambos motores, por lo que a la potencia se le añadirá y restara la deflexion de la palanca
a dicha potencia en caso de haber perturbaciones (rollOutput depende directamente de la posición angular de Roll).

Los servos redireccionan el flujo de aire de los motores, por lo que son responsables del movimiento de pitch y yaw, dependiendo de su direccion de deflexion, la tasa de
movimiento en ambos ejes aumentara o será cercana a 0 (aeronave estabilizada), como queremos que el servo trabaje con los pulsos del radio control, limitamos su movimiento
a un rango de 1000 a 2000, ya dependiendo de la direccion que queramos darle, y la cantidad de deflexion de las palancas de yaw y pitch, alterará la salida de los servos

*/

//----------------------------------------------- Estabilización del controlador PID ------------------------------------------------------

/* El código siguiente se implementá para resolver un problema mayor que tuvo el controlador PID de este proyecto en sus etapa iniciales de desarrollo

Debido a que el controlador implementado en el prototipo esta limitado por la taza de actualización del micro controlador Atmega 2650, se tienen que tener en cuenta dos cosas:

	La cantidad de tiempo que le toma al micro controlador, efectuar todos los calculos necesarios para las salidas
	La cantidad máxima de tiempo que le toma en la comunicación con el radio control

En sus etapas iniciales de desarrollo, se encontró que las librerias C++ del entorno de desarrollo Arduino, provienen de una abstracción de lenguajes de mas bajo nivel,
esto lo implementa la empresa Arduino para facilitar a los usuarios el desarrollo de prototipos y proyectos debido a su filosofia "Hazlo tu mismo", y para dejar que personas
ajenas a la profesion de electrónica, tengan una acercamiento agradable y con una curva de aprendizaje baja a dicha disciplina.
Esto, sin embargo, tiene un costo de desempeño en el microcontrolador, tomandole a procesos sumamente sencillos (como declarar pines de salida) un tiempo sumamente grande.

Debido a la (relativa) gran cantidad de cálculos que tiene que realizar un controlador de vuelo, fue necesario buscar una forma de optimizar multiples procesos.
Esta tarea lleva a desligarse un poco de la plataforma Arduino y adentrarse en el manual del microcontrolador, que aunque algo intimidante, provee se información sencilla y concisa.
El manual te explica como usar comandos como PORTX o DDRX, implementar contadores en hardware y otras tareas que el lenguaje arduino tiene muy mal optimizado.

Dicho esto, este código manipula el PWM a 250hz, debido a que el default de 500hz no es soportado por todos los ESC's, lo cual implica que tenemos un tiempo maximo de 4ms para 
efectuar todos los cálculos. Esto genera una serie de problema a la hora de trabajar con controladores PID.

El primer y mas serio problema es la dependencia del tiempo en la oarte integral y derivativa el controlador, si tomamos en cuenta el tiempo gastado en cálculos y comunicación, 
podemos ver que el muestreo del tiempo no es regular y esto ocasiona un problema para el controlador cuando no existe una perturbación, dado que se queda esperando indefinidamente
un error para poder compensarlo, esto genera un comportamiento indeseado llamado "integral windup". Como la parte integral reacciona a la acumulación del error, y la parte derivativa
al cambio en el error, el no existir un error, ocasiona que el controlador no tenga un muestreo de tiempo adecuado y se comporte de manera irregular.
Este primer problema se atacó modificando la clase "calcularPID" numerosas veces para poder establecer un muestreo constante, sin embargo, el problema no se solucionaba al no existir una manera
precisa de efectuar esto con el lenguaje del entorno de desarrollo arduino.

Finalmente se optó por tomar una medida sencilla para asegurarse que si no existian perturbaciones en una sección de tiempo, el controlador no se quedará esperando dichas
perturbaciones, sino que enviará un error nulo y continuara tomando mediciones para corregir perturbaciones en la posición angular y corregirlas.
La estretegia consistió en hacer esperar al controlador a que pasaran los 4000us sin importar cuanto tiempo sobrara de las cálculos mencinados, y una vez expirado el tiempo, reiniciara la lectura 
de perturbaciones, esto se hizo manipulando el registro de las salidas a los motores (puerto H), y usando contadores para medir dicho tiempo.
*/

	 while(micros() - loopTimer < 4000);					// Mientras no expire el tiempo, el contador sigue
	 loopTimer = micros();

	 PORTE |= B00110000;									// Existen salidas hacia los ESC y se implementa un contador para cada canal
	 timerCh1 = escLeft + loopTimer;
	 timerCh2 = escRight + loopTimer;
	 
	 while( PORTH >= 16 ){										// Una vez que expire el tiempo, se reinician las salida y el contador empieza de nuevo
	 	escLoopTimer = micros();
	 	if(timerCh1 <= escLoopTimer) PORTH &= B11101111;
	 	if(timerCh2 <= escLoopTimer) PORTH &= B11011111;
	 
	 }


// De esta manera se asegura un muestreo uniforme y el controlador actua como debe ( en teoria )


//------------------------------------------------------------------ TELEMETRIA --------------------------------------------------------------------------------------

// Declaramos los valores de las variables contenidas dentro del struct declarado al principio del código

	telemetria.telRoll = Roll;
	telemetria.telPitch = Pitch;
	telemetria.radioRoll = receiver1;
	telemetria.radioPitch = receiver2;
	telemetria.radioYaw = receiver4;
	telemetria.radioThrottle = receiver3;
	telemetria.VTOLModeActivation = receiverAux;

// Y usando la clase mencionada en al documentación de la libreria RF24, las escribimos en el modulo para enviarla al receptor
	
	tel.write(&telemetria, sizeof(telemetria));

//Escribir valores calculados en los servo motores
	servoDer.writeMicroseconds(servoRight);
	servoIzq.writeMicroseconds(servoLeft);

// La última clase, solo con fines de depuración

	//debug();

}


// A continuación se encuentran las clases usadas a lo largo de la rutina principal


void sensibilidadSensores(int gyro, int acc){			// El manual del sensor menciona que un cierto valor en binario modifica la sensibilidad

  byte gyroByte, accByte;								// Variables para almacenar el binario a escribir
  Wire.beginTransmission(direccionSensor);				// Iniciamos la comunicación en el registro de la sensibilidad del giroscopio
  Wire.write(sensibilidadGyro);							// Escribimos el valor dado en la clase
  if ( gyro == 0){										// Condicionales para el valor a escribir (4 casos)
    
    gyroByte = 0b00000000;
    }
  else if ( gyro == 1){
    													// Ver el documento del proyecto para mas información del manual del sensor
    gyroByte = 0b00001000;
    }
  else if (gyro == 2){
    
    gyroByte = 0b00010000;
    }
  else if (gyro = 3){
   
    gyroByte = 0b00011000;
    }
  else {
 
  }
  Wire.write(gyroByte);
  Wire.endTransmission();

  Wire.beginTransmission(direccionSensor);
  Wire.write(sensibilidadAcc);
  if ( acc == 0){
 														// Esta sección es esencialmente lo mismo, pero aplicado al acelerómetro
    accByte = 0b00000000;
    }
  else if ( acc == 1){
 
  accByte = 0b00001000;
  }
  else if ( acc == 2){
  
    accByte = 0b00010000;
    }
  else if ( acc == 3){
   
    accByte = 0b00011000;
    }
  else {
   
    }
  Wire.write(accByte);
  Wire.endTransmission();
}


//	 FILTRO PASABAJO

void filtroPasaBajo(int fuerza){					// En la rutina principal se define la fuerza del filtro pasabajo (mas información en el documento)
  if ( fuerza < 0 || fuerza > 6){					// cada numero del 0 al 6 indica un rango de frecuencias a filtrar (información en manual)
    fuerza = 0;
    }
  Wire.beginTransmission(direccionSensor);			// Nos comunicamos con el sensor
  Wire.write(direccionFiltroPasaBajo);				// Buscamos el registro del filtro pasabajo
  Wire.write(fuerza);								// Escribimos el rango de frecuencias seleccionado
  Wire.endTransmission();							// Terminamos la comunicación
}


// CALIBRACIÓN DEL SENSOR

/* 
La placa usada contiene el chip MPU6050 que contiene tanto acelerómetro como giroscópio (y un sensor de temperatura) de los cuales se puede obtener información 
de asi desearse, el acelerómetro devuelve datos de aceleración en gs, y el giroscópio datos en forma de dps (grados por segundo).
Para el uso que se le dará a los datos hay que tener en cuenta algunos detalles de importancia:
	La resolución de las mediciones que otorga dicho chip (el chip está contenido en la placa GY521, que en general se considera de bajo costo y por lo mismo, no posee una resolución magnìfica)
	La alta sensibilidad que posee el acelerómetro a la mínima vibración  
	La naturaleza del giroscópio a tener un error de estado estable (si se quiere obtener la posición ángular por medio de integración el 0 se va recorriendo)

Dicho esto, es normal que al usar el sensor (siendo el factor de mas importancia, el hecho de ser un sensor de bajo costo), posea errores en la medición, es por ello, que para iniciar 
las correcciones de dichos fenómenos, se implementa una rutina de calibración sencilla, en la cual al inicializar el sensor (despues de la etapa de reinicio), se hace un promedio de las mediciones
durante un periodo de tiempo para obtener el error promedio con respecto a un punto deseado, y asi añadirlo o sustraerlo a los cálculos que se efectuan para obtener la posición ángular.

*/

void calibracion(){									

  int x = 0, y = 0, z = 0, i;					// Declaramos las variables para calcular las variaciones (offsets) y un contador
  leerDatos();									// Obtenemos los datos de interes de los sensores
  x = gyroX;									// Metemos los datos de interes a las variables iniciales
  y = gyroY;
  z = gyroZ;

    for (i = 1; i <= 1000; i++){				// Tomamos mil mediciones y sacamos un promedio entre ellas
      leerDatos();
      x = (x + gyroX)/2;
      y = (y + gyroY)/2;
      z = (z + gyroZ)/2;
      }
   offsetGyroX = x;								// El promedio en cada eje, será la desviación que tiene el sensor 
   offsetGyroY = y;
   offsetGyroZ = z;

  x = accX;
  y = accY;
  z = accZ;

      for (i = 1; i <=1000; i++){				// Hacemos lo mismo para el acelerómetro, el eje Z no lo tomamos en cuenta, ya que implica tener en cuenta la gravedad
        leerDatos();							// Como no es necesario hacer una estabilización en el eje Z, no se tomará en cuenta 
        x = (x + accX)/2;
        y = (y + accY)/2;
        z = (z + accZ)/2;
        }
   offsetAccX = x;
   offsetAccY = y;



}


/*
La siguiente clase implica la comunicación i2c con el chip MPU6050, se anexa la documentación de la libreria Wire al documento del proyecto
*/

void leerDatos(){		

  Wire.beginTransmission(direccionSensor); 					// Iniciamos la comunicación con el sensor
  Wire.write(inicioDeDatos);               					// Buscamos el registro que inicia la salida de datos
  Wire.endTransmission();                 					// Terminamos la transmisión

  Wire.requestFrom(direccionSensor, 14);   					// Le requerimos el sensor, 14 bytes 
  if (Wire.available() == 14){								// NOTA: El manual del sensor indica, que las mediciones se dan en 2 partes de 8 bits (1 byte)
    accX = Wire.read() << 8 | Wire.read();					// Por lo que se usa el comando << corrimiento de bits (bitwise shift) a la izquierda en las variables para poder introducir la información de 16 bits en una sola variable
    accY = Wire.read() << 8 | Wire.read();
    accZ = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();					// Se anexa la temperatura en las variables de salida, para poder sacar el registro completo de información y no tener que hacer uno por uno
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
    }
  accXCalibrado = (float) (accX - offsetAccX);				// Se calibra la medición con los offsets de dicha clase
  accYCalibrado = (float) (accY - offsetAccY);
  accZCalibrado = (float) (accZ - offsetAccZ);
  gyroXCalibrado = (float) (gyroX - offsetGyroX);
  gyroYCalibrado = (float) (gyroY - offsetGyroY);
  gyroZCalibrado = (float) (gyroZ - offsetGyroZ); 
}


/*NOTA de la clase leerDatos()
Puede ser confuso el hecho de que la clase ya posea la calibración con las variables offset, por lo que se aclara que dicha variables al iniciar son 0, y solo se hace la calibración hasta que tenga datos que leer
*/



//Comunicación con el radio
//Aqui se definen los registros por los cuales entraran los datos de la comunicación por PWM
//Para este caso se utiliza el control de registro 2 del microcontrolador para poder usar interrupciones, dicho registro se encuentra en el puerto K
//Mas información del uso de estos puertos en el manual del microcontrolador Atmega 2650

void definirInterrupciones() {
  
   PCICR  |= B00000100;				// Usamos el control de registro 2
   PCMSK2 |= B00111111;  			// Usamos 6 entradas en el puerto K (6 canales)
  
  }

 

/*
Debido a la naturaleza del PWM, se encontró que la manera en que se ha atacado este problema ha sido usando interrupciones, de manera que cada vez que hay un cambio en el pulso
el micro controlador detecta dicho cambio en el pulso, dicho esto, existen algunas librerias de código abierto para poder usar esta caracteristica, sin embargo, le toma una cantidad
considerablemente grande al microcontrolador el efectuar dichas mediciones de los pulsos del radio control. Buscando una manera de optimizar este problema, la página de Arduino tiene
un adjunto sobre de habla del ISR (interrupt service routines) que son funciones especiales, ya que no tienen ningun parametro y no devuelven nada, tambien se menciona que trabajan en paralelo
a la rutina principal, lo que puede ahorrar una gran cantidad de tiempo si solo se usa para leer el ancho de pulso de la señal PWM.

El manual del microcontrolador menciona que el vector INT2 esta contenido en el control de registro dos, por lo que usamos dicho registro para medir el tiempo que tarda el pulso en bajar y subir,
en microsegundos, obtenemos la posición de las palancas del radio control.
Mas detalle en el documento del proyecto
*/

    ISR(PCINT2_vect){

    if (ultimoCanal1 == 0 && PINK & B00000001){					// Si el pulso estaba en bajo y el registro detecta una subida, se declara el pulso en alto y se inicia un contador
      ultimoCanal1 = 1;	
      timer1 = micros();
    }
    else if (ultimoCanal1 == 1 && !(PINK & B00000001)){			// Si el pulso se encontraba en alto y se detecta una caida, medir el tiempo que tardo en subir y bajar e introducirlo en la variable del recepetor
      ultimoCanal1 = 0;
      receiver1 = micros() - timer1;
    }															// Hacer lo mismo para todos los canales

       if (ultimoCanal2 == 0 && PINK & B00000010){
      ultimoCanal2 = 1;
      timer2 = micros();
    }
    else if (ultimoCanal2 == 1 && !(PINK & B00000010)){
      ultimoCanal2 = 0;
      receiver2 = micros() - timer2;
    }

       if (ultimoCanal3 == 0 && PINK & B00000100){
      ultimoCanal3 = 1;
      timer3 = micros();
    }
    else if (ultimoCanal3 == 1 && !(PINK & B00000100)){
      ultimoCanal3 = 0;
      receiver3 = micros() - timer3;
    }

       if (ultimoCanal4 == 0 && PINK & B00001000){
      ultimoCanal4 = 1;
      timer4 = micros();
    }
    else if (ultimoCanal4 == 1 && !(PINK & B00001000)){
      ultimoCanal4 = 0;
      receiver4 = micros() - timer4;
    }
    if (ultimoCanalAux == 0 && PINK & B00010000){
      ultimoCanalAux = 1;
      timerAux = micros();
    }
    else if (ultimoCanalAux == 1 && !(PINK & B00010000)){
      ultimoCanalAux = 0;
      receiverAux = micros() - timerAux;
    }
    if (ultimoCanalVTOL == 0 && PINK & B00100000){
      ultimoCanalVTOL = 1;
      timerVTOL = micros();
    }
    else if (ultimoCanalVTOL == 1 && !(PINK & B00100000)){
      ultimoCanalVTOL = 0;
      receiverVTOL = micros() - timerVTOL;
    }
  }
 

 // Solo para propósitos de desarrollo y testeo

 void debug() {

  Serial.print("Roll: ");
  Serial.print(Roll); 
  Serial.print(" Pitch: ");
  Serial.print(Pitch);  
  Serial.print(" ESCLEFT: ");
  Serial.print(escLeft); 
  Serial.print(" ESCRIGHT: "); 
  Serial.print(escRight);
  Serial.print(" ServoLeft: ");
  Serial.print(servoLeft); 
  Serial.print(" ServoRight: "); 
  Serial.print(servoRight); 
  Serial.print(" RadioRoll ");
  Serial.print(receiver1); 
  Serial.print(" RadioPitch ");
  Serial.print(receiver2); 
  Serial.print(" Throttle  ");
  Serial.print(throttle); 
  Serial.print(" RadioYaw ");
  Serial.print(receiver4); 
  Serial.print(" ARMED ");
  Serial.print(start); 
  Serial.print(" VTOL MODE ");
  Serial.print(receiverVTOL);
  Serial.print(" FailsafeFlag ");
  Serial.println(failsafeFlag);
  }



// Clase creada para efectuar los cálculos del controlador PID

/*
Se explicó previamente un poco del proceso de la construcción de esta clase, y al resolver los problemas del muestreo del tiempo es posible hacer esta clase sumamente sencilla
usando explicitamente solo la ecuación del controlador PID
*/

void calcularPID(){
  
   errorRoll = gyroXCalibrado - rollSetpoint;							// Se calcula el error para la parte proporcional
   rollIntegralError += rollKi*errorRoll;					// Se hace una sumatoria continua del error para la parte integral ( e integramos la ganancia de una vez )
   
   if(rollIntegralError > rollPIDMax) {						// Limitamos la acumulación del error máxima y mínima del controlador para evitar comportamientos bruscos e indeseados
      rollIntegralError = rollPIDMax;
    }
   else if(rollIntegralError < rollPIDMax* -1) {
      rollIntegralError = rollPIDMax*-1;
    }
    rollOutput = (rollKp * errorRoll) + (rollIntegralError) + (rollKd* (errorRoll - rollUltimoError));		// Ecuación del controlador PID para un eje
    
 // Podemos observar que en la parte derivativa se cálcula el cambio en el error a partir de la medición anterior del mismo  (que al inicio es 0)


  if ( rollOutput > rollPIDMax){				// Limitamos la salida del controlador PID para evitar movimientos indeseados 
      rollOutput = rollPIDMax;
    } 
  else if(rollOutput < rollPIDMax*-1)  {
      rollOutput = rollPIDMax*-1;
  }
    rollUltimoError = errorRoll;				// Actualizamos el error



 // Se efectua el mismo cálculo para los ejes restantes

  errorPitch = gyroYCalibrado - pitchSetpoint;
  pitchIntegralError += pitchKi*errorPitch;
  if(pitchIntegralError > pitchPIDMax){
        pitchIntegralError = pitchPIDMax;
      }
   else if (pitchIntegralError < pitchPIDMax*-1){
      pitchIntegralError = pitchPIDMax*-1;
    }

  pitchOutput = ((pitchKp*errorPitch) + (pitchIntegralError) + (pitchKd * (errorPitch - pitchUltimoError))) * -1;
  if( pitchOutput > pitchPIDMax){
      pitchOutput = pitchPIDMax;
    }
  else if(pitchOutput < pitchPIDMax* -1){
      pitchOutput = pitchPIDMax * -1;
    }
  pitchUltimoError = errorPitch;

  errorYaw = gyroZCalibrado - yawSetpoint;
  yawIntegralError += yawKi*errorYaw;
  if(yawIntegralError > yawPIDMax){
      yawIntegralError = yawPIDMax;  
  }
  else if( yawIntegralError < yawPIDMax* -1){
      yawIntegralError = yawPIDMax * -1;
    }

  yawOutput = (yawKp*errorYaw) + (yawIntegralError) + (yawKd* (errorYaw - yawUltimoError));
  if (yawOutput > yawPIDMax){
      yawOutput = yawPIDMax;      
    }
  else if( yawOutput < yawPIDMax * -1){ 
        yawOutput = yawPIDMax * -1;
    }
   yawUltimoError = errorYaw;
  }
  

/*

NOTAS DEL AUTOR:
Quiero agradecer al proyecto Cleanflight, por tener una " remasterización en limpio " del proyecto de código abierto Multiwii, su documentación fue de mucha ayuda para
resolver una serie de dudas respecto al funcionamiento de los registros para la comunicación con el radio.

Espero que abra puertas a alumnos interesados en el área de control de aeronaves para seguir desarollando
el área dentro del Instituto Politécnico Nacional.

Repitiendo lo mencionado anteriormente, este proyecto no es algo innovador si consideramos la complejidad que tiene para alguien con bases en mecatrónica y control automático, 
sin embargo, quiero hacer enfásis en el reto que fue aprender múltiples temas de desconocimiento total o parcial para el programa de aeronáutica.
También quiero hacer enfasis en que los alumnos deberían darse cuenta de las grandes capacidades y sobre todo las limitaciones de la plataforma Arduino, yo consideraria
como principal ventaja, el increiblemente rápido tiempo para desarrollar prototipos pero no lo consideraria adecuado para producción, lo cual es un tema bastante sensible debido a su 
amplio uso en startups de tecnologia que planean crecer como empresa, remarco, para iniciar una etapa de desarrollo y prototipado rápido es una gran herramienta, mas consideraria usar
el microcontrolador solo (y aprender a usarlo con, por ejemplo, Atmel Studio) de querer pasar a una etapa de producción.

Como autor espero, que la comunidad de Aeronaves No tripuladas o drones, siga reciendo dentro la unidad UPIIG, ya que es un área de interes actual ademas de ser sumamente entretenida.
También espero despertar el interés de algunos pocos alumnos que esten interesados en el desarrollo de software y firmware para fines aeronauticos, ya que es un área escasa o nula 
en nuestro país (y dominada por otras áreas de la ingeniería)


*/

