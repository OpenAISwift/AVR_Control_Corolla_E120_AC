/*INICIO DE LIBRERIAS*/
#include <Arduino.h>
#include "MeanFilterLib.h"
#include "pins.h"
#include "macros.h"
#include "constant.h"
#include "utility.h"
#include "cactus_io_DHT22.h"
#include "Bounce2.h"
#include "Servo.h"
#include <math.h>
/*FIN DE LIBRERIAS*/

/*DECLARACION DE CLASES*/
Servo servoAirMix;
Servo servoVentMode;
DHT22 dht(Dht_Room);
Bounce posicionLuz = Bounce();
/*DECLARACION DE CLASES*/

/* INICO DEFINICIONES DE VARIABLES */

/*
BOOL1 = Temperatura Evaporador
BOOL2 = Temperatura Ambiente
BOOL3 = Luz Ambiental
BOOL4 = Temperatura Interior
BOOL5 = Humedad Interior
BOOL6 = Temperatura Sensacion Interior
BOOL7 = Punto de rocio
BOOL8 = 
*/
uint8_t Sta_MessageSensors;

/*
BOOL1 = Preostato interruptor simple (Proteccion Aire Acondicionado)
BOOL2 = Preostato interruptor doble (Proteccion Aire Acondicionado)
BOOL3 = Control ACT ECU
BOOL4 = Encendido/Apagado Compresor segun la temperatura del evaporador
BOOL5 = Estado Compresor - Encendido/Apagado
BOOL6 = Envio mensaje interruptor simple
*/
uint8_t Sta_Compressor;

/*
BOOL1 = Envio Mensaje Compresor
BOOL2 = Envio mensaje interruptor simple
BOOL3 = Envio mensaje interruptor doble
BOOL4 = Envio mensaje ACT ECU
BOOL5 = 
BOOL6 = 
BOOL7 = 
BOOL8 = 
*/
uint8_t Sta_MensajeCompressor;

/*
BOOL1 = Interruptor de Iluminacion Interior
BOOL2 = Heater Relay
BOOL3 = Compresor Relay Fan1
BOOL4 = Temperatura Relay Fan1
*/
uint8_t Sta_Switch;

/*
BOOL1 = Mensaje Interruptor de la Iluminacion
BOOL2 = Mensaje Relay Fan1
BOOL3 = Mensaje de Blower
*/
uint8_t Sta_MessageSwitch;

/*
BOOL1 = Control Automatico
BOOL2 = Control de envio de datos
BOOL3 = Control de recepcion de datos
BOOL4 = Control de recepcion de datos en progreso
*/
uint8_t Sta_Auto;

/*
Variables de sensores del sistema de Aire Acondicionado
*/
int8_t Temp_Evaporador; // Temperatura Evaporador
int8_t Temp_Ambiente;	// Temperatura Ambiente
int8_t Temp_Interior;	// Temperatura Interior
int8_t Temp_SInterior;	// Sensacion Temperatura Interior
int8_t Temp_DewPoint;	// Punto de rocio
uint8_t Humi_Interior;	// Humedad Interior
uint8_t Illu_Ambiental; // Fotodiodo

char mensajeDesdePc[Len_BufferInt]; // Vector Mensaje PC
int enteroDesdePc = 0;				// Comando desde HOST

uint8_t Val_Blower = 0; // Valor del Blower(Soplador de Aire Acondicionado)
uint8_t Val_AirMix = 0; // Valor de la posicion del servo de mezcla de aire(Air Mix)

uint8_t Est_AcAac = 255; // Estado para el control de encendido del compresor del aire acondicionado
uint8_t Est_ADese = 255; // Estado para el desempañamiento automatico parabrisa delantera

unsigned long Tim_Current = 0; // Temporizador desde el inicio del sistema
/* FIN DEFINICIONES DE VARIABLES */

/*FUNCIONES PROTOTIPO*/
void Fun_ReadSerial();
void lecturaSensores();
void Fun_ActMessage();
void Con_Automatic();
void Con_Compressor();

void analogReadings();
/*FUNCIONES PROTOTIPO*/

void controlManual()
{
	/************************************************************************/
	/* LISTA DE COMANDOS DISPONIBLES                                        */
	/************************************************************************/
	/*
	<a,"valor"> ->	Control de la compresor del sistema del aire
	acondicionado ( 1 = Activado, 0 = desactivado )
	<b,"valor"> ->	Control de la velocidad del blower del sistema del aire
	acondicionado ( valor = velocidad)
	<c,"valor"> ->	Control del ventilador de refrigeracion del motor y
	aire acondicionado ( 1 = Encendido, 0 = Apagado)
	<d,"valor"> ->	Control del desempañamiento de la parabrisa posterior
	( 1 = Encendido, 0 = Apagado)
	<e,"valor"> -> Desempañador parabrisa delantera con aire acondicionado
	( 1 = Encendido, 0 = Apagado)
	<f,"valor"> ->	Posicion de la entrada de aire ( 1 = Recirculacion,
	0 = Aire externo
	<v,"valor"> ->	Posicion del servo de control de las regillas para el
	control del modo de circulación de aire
	<m,"valor"> ->	Posicion del servo de control de las regillas para la
	mezcla de aire caliente y frio límite inferior de 15° y superior de 135°
	<w,"valor"> ->	Control de la señal de parada( 1 = Encendido, 0 = Apagado)
	<z, "valor">->	Control de estado automatico
	*/

	switch (mensajeDesdePc[0])
	{

	case 'a': // Encendido del compresor del aire acondicionado
		if (enteroDesdePc == 1)
		{
			Est_AcAac = 1;
			break;
		}
		else
		{
			Est_AcAac = 0;
			break;
		}
		break;

	case 'b': // Control del Pwm_Blower del sistema del aire acondicionado
		if (enteroDesdePc >= 1)
		{
			if (!(Sta_Switch & BOOL2))
			{
				Sta_Switch |= BOOL2;
				digitalWrite(Rel_Heater, HIGH);
			}
		}
		else
		{
			if (Sta_Switch & BOOL2)
			{
				if (Est_AcAac != 255)
				{
					Est_AcAac = 0; // Desactiva el compresor del aire acondicionado
				}
				Sta_Switch &= ~BOOL2;
				digitalWrite(Rel_Heater, LOW);
			}
		}
		Val_Blower = map(enteroDesdePc, 0, 6, 0, 255);
		analogWrite(Pwm_Blower, Val_Blower);
		break;

	case 'd': // Control del desempañamiento de la parabrisa trasera
		if (enteroDesdePc == 1)
		{
			digitalWrite(Rel_Defroster, HIGH);
			break;
		}
		else
		{
			digitalWrite(Rel_Defroster, LOW);
			break;
		}

	case 'e': // Control del desempañador Parabrisas delantero
		if (enteroDesdePc == 1)
		{
			Est_ADese = 2; // Inicia el desempañador manual
			break;
		}
		else
		{
			if (Sta_Auto & BOOL1) // Comprueba si esta activado el control automatico
			{
				Est_ADese = 1; // Inicia el desempañamiento automatico
				break;
			}
			else
			{
				Est_ADese = 0; // Desactiva el desempañador
				break;
			}
		}

	case 'f': // Control de recirculacion del sistema del aire acondicionado
		if (enteroDesdePc == 1)
		{
			digitalWrite(Dig_Recirculation, HIGH);
			digitalWrite(Dig_AirFresh, LOW);
			break;
		}
		else
		{
			digitalWrite(Dig_Recirculation, LOW);
			digitalWrite(Dig_AirFresh, HIGH);
			break;
		}

	case 'v': // Control de posicion de las ventanas del aire acondicionado
		switch (enteroDesdePc)
		{
		case 1:
			servoVentMode.write(0); // Posicion parabrisa
			break;
		case 2:
			break;
		case 3:
			servoVentMode.write(90); // Posicion pies
			break;
		case 4:
			break;
		case 5:
			servoVentMode.write(180); // Posicion Frente
			break;

		default: // Caso por defecto
			break;
		}

		break;

	case 'm': // Control de la mescla de aire frio y aire caliente
		if (enteroDesdePc <= 6 && enteroDesdePc >= 0)
		{
			Val_AirMix = map(enteroDesdePc, 0, 6, 15, 135);
			servoAirMix.write(Val_AirMix);
		}
		break;

	case 'z': //Control de estado automatico
		if (enteroDesdePc == 1)
		{
			Sta_Auto |= BOOL1; // Activamos el funcionamiento automatico
			Est_ADese = 1;	   // Inicia el desempañamiento automatico (Estado de Transicion)
			break;
		}
		else
		{
			Sta_Auto &= ~BOOL1; // Desactivamos el funcionamiento automatico
			Est_ADese = 0;		// Desactiva el desempañamiento automatico
			break;
		}
		break;

	case 'w': // Encender luces de emergencia
		if (enteroDesdePc == 1)
		{
			digitalWrite(Rel_WarningLight, HIGH);
			break;
		}
		else
		{
			digitalWrite(Rel_WarningLight, LOW);
			break;
		}
		break;

	default: // Caso por defecto no haga nada
		break;
	}
}
void Con_AutoCli() // Funcion para el control automatico del climatizador
{
	/* 
	BOOl1 = Encendido o apagado segun el punto de rocio
	*/
	static uint8_t Est_CAuto; // Control Automatico del climatizador

	switch (Est_ADese)
	{
	case 0: // Desactiva el desempañamiento
		if (Est_AcAac != 255)
		{
			Est_AcAac = 0; // Desactiva el compresor del aire acondicionado
		}
		if (Val_Blower != 0)
		{
			/*ENVIOS DE MENSAJES*/
			Sta_Auto |= BOOL2;
			Sta_MessageSwitch |= BOOL3;
		}

		Val_Blower = map(0, 0, 6, 0, 255);	   // Asigna la velocidad del Pwm_Blower
		digitalWrite(Rel_Heater, LOW);		   // Desactiva el relay del conjunto del aire acondicionado
		analogWrite(Pwm_Blower, Val_Blower);   // Asigna el pwm del Pwm_Blower
		digitalWrite(Dig_Recirculation, HIGH); // Activa la recirculacion
		digitalWrite(Dig_AirFresh, LOW);	   // Desactiva la entrada de aire desde afuera
		Est_ADese = 255;					   // Cambia al siguiente estado

		break;

	case 1: // Estado de transicion desempañamiento automatico

		if (Est_AcAac != 255)
		{
			Est_AcAac = 0; // Desactiva el compresor del aire acondicionado
		}
		if (Val_Blower != 0)
		{
			/*ENVIOS DE MENSAJES*/
			Sta_Auto |= BOOL2;
			Sta_MessageSwitch |= BOOL3;
		}
		Val_Blower = map(0, 0, 6, 0, 255);	   // Asigna la velocidad del Pwm_Blower
		digitalWrite(Rel_Heater, LOW);		   // Desactiva el relay del conjunto del aire acondicionado
		analogWrite(Pwm_Blower, Val_Blower);   // Asigna el pwm del Pwm_Blower
		digitalWrite(Dig_Recirculation, HIGH); // Activa la recirculacion
		digitalWrite(Dig_AirFresh, LOW);	   // Desactiva la entrada de aire desde afuera
		Est_ADese = 3;						   // Cambia al siguiente estado
		break;

	case 2: // Caso 1 para el inicio de desempañamiento manual

		Est_AcAac = 1;						  // Activa el compresor del aire acondicionado
		digitalWrite(Rel_Heater, HIGH);		  // Activa el relay del conjunto del aire acondicionado
		Val_Blower = map(3, 0, 6, 0, 255);	  // Asigna la velocidad del Pwm_Blower
		analogWrite(Pwm_Blower, Val_Blower);  // Asigna el pwm del Pwm_Blower
		servoVentMode.write(0);				  // Posiciona la regilla para la parabrisa delantera
		digitalWrite(Dig_Recirculation, LOW); // Desactiva la recirculacion
		digitalWrite(Dig_AirFresh, HIGH);	  // Activa la entrada de aire desde afuera
		Est_ADese = 255;					  // Cambia al siguiente estado

		/*ENVIOS DE MENSAJES*/
		Sta_Auto |= BOOL2;
		Sta_MessageSwitch |= BOOL3;
		break;

	case 3: // Inicia el desempañador automatico

		Est_CAuto |= BOOL1; // Incia la variable para encendido de climatizador segun el punto de rocio
		Est_ADese = 4;		// Cambia al siguiente estado
		break;

	case 4: // Contorl de encendido y apagado del sistema de aire acondicionado

		if (Est_CAuto & BOOL1) // Control de encedido del climatizador segun el punto de rocio
		{
			if (Temp_Ambiente < Temp_DewPoint) // Si la temperatura ambiente es inferior al punto de rocio
			{
				Est_CAuto &= ~BOOL1;			   // Cambia el control del climatizador para el apagado del climatizador
				Est_AcAac = 1;					   // Activa el compresor del aire acondicionado
				digitalWrite(Rel_Heater, HIGH);	   // Activa el conjunto del climatizador
				Val_Blower = map(3, 0, 6, 0, 255); // Asigna la velocidad de 3 al ventilador
				analogWrite(Pwm_Blower, Val_Blower);
				digitalWrite(Dig_Recirculation, LOW); // Desactiva la recirculacion
				digitalWrite(Dig_AirFresh, HIGH);	  // Activa la entrada de aire desde afuera
				servoVentMode.write(0);				  // Dirige todo el aire al parabrisa
				//Val_AirMix = map(3, 0, 6, 15, 135);	// Asigna entre la mescla de aire caliente y frio en 2
				//servoAirMix.write(Val_AirMix);

				/*ENVIOS DE MENSAJES*/
				Sta_Auto |= BOOL2;
				Sta_MessageSwitch |= BOOL3;
				break;
			}
			break;
		}
		else
		{
			if (Temp_Ambiente > Temp_DewPoint + 1)
			{
				Est_CAuto |= BOOL1; // Cambia el control del climatizador para el encendido del climatizador
				Est_AcAac = 0;		// Desactiva el compresor del aire acondicionado
				Val_Blower = 0;
				analogWrite(Pwm_Blower, Val_Blower);
				digitalWrite(Rel_Heater, LOW);
				//Val_AirMix = map(0, 0, 6, 15, 135);	// Asigna entre la mescla de aire caliente y frio en 0
				//servoAirMix.write(Val_AirMix);

				/*ENVIOS DE MENSAJES*/
				Sta_Auto |= BOOL2;
				Sta_MessageSwitch |= BOOL3;
				break;
			}
			break;
		}
		break;

	default: // Caso por defecto no haga nada
		break;
	}
	//////////////////////////////////////////////////////////////////////////
}

void setup()
{
	Serial.begin(Spe_Serial);

	dht.begin();
	Upd_FunParameter();

	pinMode(Pwm_Blower, OUTPUT);
	pinMode(Rel_Heater, OUTPUT);
	pinMode(Rel_Defroster, OUTPUT);
	pinMode(Dig_Recirculation, OUTPUT);
	pinMode(Dig_AirFresh, OUTPUT);
	pinMode(Rel_Cluch, OUTPUT);
	pinMode(Rel_Fan1, OUTPUT);
	pinMode(Dig_Ac1, OUTPUT);
	pinMode(Rel_WarningLight, OUTPUT);

	pinMode(Swi_Ill, INPUT);
	pinMode(Swi_SingleAC, INPUT);
	pinMode(Swi_DualAC, INPUT);
	pinMode(Dig_Act, INPUT);

	servoAirMix.attach(Ser_AirMix);
	servoVentMode.attach(Ser_VentMode);
	posicionLuz.attach(Swi_Ill);
	posicionLuz.interval(5);
	Val_AirMix = map(0, 0, 6, 15, 135);
	servoAirMix.write(Val_AirMix);

	/*ACCIONES INICIALES DEL PROGRAMA*/
	Sta_Switch |= BOOL4; // Activa Relay Fan1
}

void loop()
{
	Tim_Current = millis(); // Actualiza el tiempo para los temporizadores
	Con_Automatic();
	lecturaSensores();
	Fun_ReadSerial();
	Con_AutoCli();
	Con_Compressor();
	Fun_ActMessage();

	analogReadings();
}

void Con_Compressor() // Funcion para el control del compresor del aire acondicionado
{
	/*
	Tim_AntActivationAC = Temporizador activacion de aire acondicionado
	*/
	static unsigned long Tim_AntActivationAC = 0;

	/*
	Lectura del interruptor simple del sistema del aire acondicionado 
	(1 = Cerrado 0 = Abierto) 
	Abierdo no debe encender el aire acondicionado
	*/
	if (digitalRead(Swi_SingleAC))
	{
		if (!(Sta_Compressor & BOOL1))
		{
			Sta_Compressor |= BOOL1;

			/*ENVIO DE MENSAJES*/
			Sta_Auto |= BOOL2;
			Sta_MessageSwitch |= BOOL2;
			//Sta_MensajeCompressor |= BOOL2;
		}
	}
	else
	{
		if (Sta_Compressor & BOOL1)
		{
			Sta_Compressor &= ~BOOL1;

			/*ENVIO DE MENSAJES*/
			Sta_Auto |= BOOL2;
			Sta_MessageSwitch |= BOOL2;
			//Sta_MensajeCompressor |= BOOL2;
		}
	}

	/*
	Lectura del interruptor doble del sistema del aire acondicionado 
	(0 = Cerrado 1 = Abierto) 
	Abierdo no debe encender el aire acondicionado
	*/
	if (digitalRead(Swi_DualAC))
	{
		if (!(Sta_Compressor & BOOL2))
		{
			Sta_Compressor |= BOOL2;

			/*ENVIO DE MENSAJES*/
			// Sta_Auto |= BOOL2;
			// Sta_MensajeCompressor |= BOOL3;
		}
	}
	else
	{
		if (Sta_Compressor & BOOL2)
		{
			Sta_Compressor &= ~BOOL2;
			// Sta_Auto |= BOOL2;
			// Sta_Compressor |= BOOL7;
		}
	}

	/*
	Lectura de la señal de ECU para desactivacion del compresor del aire acondicionado 
	(1 = Funcionamineto normal 0 = Apagar compresor)
	*/
	if (digitalRead(Dig_Act))
	{
		if (!(Sta_Compressor & BOOL3))
		{
			Sta_Compressor |= BOOL3;

			/*ENVIO DE MENSAJES*/
			// Sta_Auto |= BOOL2;
			// Sta_MensajeCompressor |= BOOL4;
		}
	}
	else
	{
		if (Sta_Compressor & BOOL3)
		{
			Sta_Compressor &= ~BOOL3;

			/*ENVIO DE MENSAJES*/
			// Sta_Auto |= BOOL2;
			// Sta_MensajeCompressor |= BOOL4;
		}
	}

	switch (Est_AcAac)
	{
	case 0: // Desactivacion del sistema del compresor del aire acondicionado

		digitalWrite(Dig_Ac1, LOW);
		digitalWrite(Rel_Cluch, LOW);
		Sta_Switch &= ~BOOL3;	  // Desactiva el relay del ventilador de refigeracion del radiador
		Sta_Compressor &= ~BOOL5; // Cambio Estado de Compresor
		Est_AcAac = 255;		  // Cambia al siguiente estado

		/*ENVIO DE MENSAJES*/
		Sta_Auto |= BOOL2;				// Activacion de envio de datos
		Sta_MessageSwitch |= BOOL2;		// Envio de datos de estado FAN1
		Sta_MensajeCompressor |= BOOL1; // Envio mensaje de Estado de Compresor
		break;

	case 1: // Inicia la activacion del compresor del sistema del aire acondicionado

		Tim_AntActivationAC = Tim_Current; // Actualiza el temporizador
		digitalWrite(Dig_Ac1, HIGH);	   // Activa la salida de control de la ECU para aumentar el ralentí
		Sta_Switch |= BOOL3;			   // Activa el relay del ventilador de refigeracion del radiador
		Sta_Compressor |= BOOL5;		   // Cambio Estado de Compresor
		Est_AcAac = 2;					   // Cambia al siguiente estado

		/*ENVIO DE MENSAJES*/
		Sta_Auto |= BOOL2;				// Activacion de envio de datos
		Sta_MessageSwitch |= BOOL2;		// Envio de datos de estado FAN1
		Sta_MensajeCompressor |= BOOL1; // Envio mensaje de Estado de Compresor
		break;

	case 2: // Espera de tiempo asignado a Tim_AntActivationAC para la espera de encendido y comprobacion del compresor del aire acondicionado
		if (Tim_Current - Tim_AntActivationAC >= Int_ActE)
		{
			Sta_Compressor |= BOOL4; // Reinicia el estado de encendido del del compresor por temperatura
			Est_AcAac = 3;			 // Cambia al siguiente estado
			break;
		}
		else
		{
			break;
		}

	case 3:
		if (!(Sta_Compressor & BOOL2))
		{
			if (!(Sta_Compressor & BOOL1))
			{
				if (Sta_Compressor & BOOL3)
				{
					if (Sta_Compressor & BOOL4)
					{ // Control del encendido del compresor del aire acondicionado por umbral de temperatura del evaporador
						if (Temp_Evaporador >= Val_UmAlt)
						{								   // Control de encendido cuando la temperatura umbral es superior o igual a la definida
							Sta_Compressor &= ~BOOL4;	   // Cambio de estado para la activacion de temperatura por debajo o igual al valor umbral
							digitalWrite(Rel_Cluch, HIGH); // Activa el embrague magnetico del compresor del aire acondicionado
							break;
						}
						break;
					}
					else
					{
						if (Temp_Evaporador <= Val_UmBaj) // Control de apagado cuando la temperatura umbral es inferior o igual a la definida
						{
							Sta_Compressor |= BOOL4;	  // Cambio de estado para la activacion de temperatura superior o igual al valor umbral
							digitalWrite(Rel_Cluch, LOW); // Desactiva el embrague magnetico del compresor del aire acondicionado
							break;
						}
						break;
					}
					break;
				}
				else
				{
					Est_AcAac = 4; // Cambia al siguiente estado
					break;
				}
				break;
			}
			else
			{
				Est_AcAac = 0; // Desactiva el compresor del aire acondicionado por falla del interruptor simple del sistema
				break;
			}

			break;
		}
		else
		{
			Est_AcAac = 0; // Desactiva el compresor del aire acondicionado por falla del interruptor doble del sistema
			break;
		}

	case 4: // Cuando la ECU ordena apagar el compresor del aire acondicionado

		digitalWrite(Rel_Cluch, LOW);	   // Desactiva el embrague magnetico del compresor del aire acondicionado
		Tim_AntActivationAC = Tim_Current; // Actualiza el temporizador
		Est_AcAac = 2;					   // Cambia al siguiente estado
		break;

	default: // Caso por defecto no haga nada
		break;
	}
}
void Con_Automatic()
{
	posicionLuz.update();

	/*
	Lectura del interruptor de pocicion de luces del tablero
	(1 = Encendido 0 = Apagado) 
	*/
	if (posicionLuz.read())
	{
		if (!(Sta_Switch & BOOL1))
		{
			Sta_Switch |= BOOL1;

			/*ENVIO MENSAJE*/
			Sta_Auto |= BOOL2;
			Sta_MessageSwitch |= BOOL1;
		}
	}
	else
	{
		if (Sta_Switch & BOOL1)
		{
			Sta_Switch &= ~BOOL1;

			/*ENVIO MENSAJE*/
			Sta_Auto |= BOOL2;
			Sta_MessageSwitch |= BOOL1;
		}
	}
	digitalWrite(Rel_Fan1, ((Sta_Compressor & BOOL1) || (Sta_Switch & BOOL3) || (Sta_Switch & BOOL4)));
}
void Fun_ReadSerial()
{
	static char receivedChars[Len_BufferInt]; // Vector Caracteres recivios

	if (Serial.available() > 0 && !(Sta_Auto & BOOL3))
	{

		char Val_RecivedChar = Serial.read();
		if (Sta_Auto & BOOL4)
		{
			static uint8_t Val_AuxSerial = 0;
			if (Val_RecivedChar != End_Trama)
			{
				receivedChars[Val_AuxSerial] = Val_RecivedChar;
				Val_AuxSerial++;
				if (Val_AuxSerial >= Len_BufferInt)
				{
					Val_AuxSerial = Len_BufferInt - 1;
				}
			}
			else
			{
				receivedChars[Val_AuxSerial] = '\0'; // Termina la cadena
				Val_AuxSerial = 0;
				Sta_Auto |= BOOL3;
				Sta_Auto &= ~BOOL4;
			}
		}
		else if (Val_RecivedChar == Ini_Trama)
		{
			Sta_Auto |= BOOL4;
		}
	}
	if (Sta_Auto & BOOL3)
	{
		static char tempChars[Len_BufferInt]; // Vector tenporal para almacenar caracteres

		strcpy(tempChars, receivedChars);
		/* Divide los datos en sus partes*/
		/////////////////////////////////////////////////////////////////////////////////////////////////
		char *strtokIndx;					 // Es utilizado por strtok() como un indice
		strtokIndx = strtok(tempChars, ","); // Optiene la primera parte de la cadena
		strcpy(mensajeDesdePc, strtokIndx);	 // Copia en mensajeDesdePc
		strtokIndx = strtok(NULL, ",");		 // Contionua donde quedo la llamada anterior
		enteroDesdePc = atoi(strtokIndx);	 // Convierte a un numero entero
		/////////////////////////////////////////////////////////////////////////////////////////////////
		controlManual();

		Sta_Auto &= ~BOOL3;
	}
}
void lecturaSensores()
{

	static unsigned long Tim_PreDigital = 0; // Tiempo anterior lectura temperatura sensores digitales

	if (Tim_Current - Tim_PreDigital >= Int_LecDigital)
	{
		static int8_t Temp_AntInterior = 0;	 // Variable temperatura Anterior Interior
		static uint8_t Humi_AntInterior = 0; // Variable humedad Anterior Interior
		static int8_t Temp_AntSInterior = 0; // Variable  Sensacion Anterior temperatura Interior
		static int8_t Temp_AntDewPoint = 0;	 // Valor Anterior de Punto de rocio
		dht.readHumidity();
		dht.readTemperature();
		Temp_AntInterior = dht.temperature_C;
		Humi_AntInterior = dht.humidity;
		Temp_AntSInterior = dht.computeHeatIndex_C();
		Temp_AntDewPoint = Fun_DewPoint(Temp_Interior, Humi_Interior);

		if (Temp_AntInterior != Temp_Interior)
		{
			Temp_Interior = Temp_AntInterior;
			Sta_Auto |= BOOL2;
			Sta_MessageSensors |= BOOL4;
		}
		if (Humi_AntInterior != Humi_Interior)
		{
			Humi_Interior = Humi_AntInterior;
			Sta_Auto |= BOOL2;
			Sta_MessageSensors |= BOOL5;
		}
		if (Temp_AntSInterior != Temp_SInterior)
		{
			Temp_SInterior = Temp_AntSInterior;
			Sta_Auto |= BOOL2;
			Sta_MessageSensors |= BOOL6;
		}
		if (Temp_AntDewPoint != Temp_DewPoint)
		{
			Temp_DewPoint = Temp_AntDewPoint;
			Sta_Auto |= BOOL2;
			Sta_MessageSensors |= BOOL7;
		}

		Tim_PreDigital = Tim_Current;
	}
}

/*Funcion para la lectura de sensores analogicos*/
void analogReadings()
{
	/*Variables locales para la lectura de sensores analogicos*/

	static int8_t tempEvaporadorOld = 0;  // Temperatura Evaporador
	static int8_t tempAmbientOld = 0;	  // Temperatura Ambiente
	static uint8_t Illu_AntAmbiental = 0; // Valor Fotodiodo

	static unsigned long Tim_PreTermistores = 0; // Tiempo anterior lectura temperatura sensores analogos
	if (Tim_Current - Tim_PreTermistores >= Int_LecTermistores)
	{
		// Calculo de la lectura del sensor del evaporador
		// Instanciar filtro media movil con ventana tamaño 3 para el sensor de temperatura del evaporador
		static MeanFilter<float> tempEvaporadorFilter(4);
		// Instanciar filtro media movil con ventana tamaño 3 para el sensor de temperatura ambiente
		static MeanFilter<float> tempAmbientFilter(4);

		Illu_AntAmbiental = map(analogRead(Fot_Solar), 0, 1023, 0, 100); // Convercion del voltage a iluminacion ambiental
		tempEvaporadorOld = tempEvaporadorFilter.AddValue(Fun_ConTemperature(analogRead(Ter_Evaporador), Ae, Be, Ce));
		tempAmbientOld = tempAmbientFilter.AddValue(Fun_ConTemperature(analogRead(Ter_Ambient), Aa, Ba, Ca));
		if (tempEvaporadorOld != Temp_Evaporador)
		{
			Temp_Evaporador = tempEvaporadorOld;
			Sta_Auto |= BOOL2;
			Sta_MessageSensors |= BOOL1;
		}
		if (tempAmbientOld != Temp_Ambiente)
		{
			Temp_Ambiente = tempAmbientOld;
			Sta_Auto |= BOOL2;
			Sta_MessageSensors |= BOOL2;
		}

		if (Illu_AntAmbiental != Illu_Ambiental)
		{
			Illu_Ambiental = Illu_AntAmbiental;
			Sta_Auto |= BOOL2;
			Sta_MessageSensors |= BOOL3;
		}
		Tim_PreTermistores = Tim_Current; // Actualiza el tiempo para la siguiente lectura
	}
}

void Fun_ActMessage() // Funcion para enviar los datos de los sensores
{
	/************************************************************************/
	/* IDENTIFICACION DE LOS SENSORES                                       */
	/************************************************************************/
	/*
	-Te = Temperatura Evaporador
	-Ta = Temperatura Ambiente
	-La = Iluminacion Ambietal
	-Ti = Temperatura Interior
	-H = Humedad Interior
	-Ts = Temperatura sensacion Interior
	-Pr = Punto Rocio
	-I = Iluminacion Interior
	-F1 = Relay Fan1
	-Co = Compresor A/C
	-Bl = Valor soplador
	*/

	String Aux_MMessage;
	if (Sta_Auto & BOOL2)
	{
		Aux_MMessage += Ini_Trama;
		Aux_MMessage += " ";
		if (Sta_MessageSensors & BOOL1)
		{
			Sta_MessageSensors &= ~BOOL1;
			Aux_MMessage += "Te:";
			Aux_MMessage += Temp_Evaporador;
			Aux_MMessage += " ";
		}
		if (Sta_MessageSensors & BOOL2)
		{
			Sta_MessageSensors &= ~BOOL2;
			Aux_MMessage += "Ta:";
			Aux_MMessage += Temp_Ambiente;
			Aux_MMessage += " ";
		}
		if (Sta_MessageSensors & BOOL3)
		{
			Sta_MessageSensors &= ~BOOL3;
			Aux_MMessage += "La:";
			Aux_MMessage += Illu_Ambiental;
			Aux_MMessage += " ";
		}
		if (Sta_MessageSensors & BOOL4)
		{
			Sta_MessageSensors &= ~BOOL4;
			Aux_MMessage += "Ti:";
			Aux_MMessage += Temp_Interior;
			Aux_MMessage += " ";
		}
		if (Sta_MessageSensors & BOOL5)
		{
			Sta_MessageSensors &= ~BOOL5;
			Aux_MMessage += "H:";
			Aux_MMessage += Humi_Interior;
			Aux_MMessage += " ";
		}
		if (Sta_MessageSensors & BOOL6)
		{
			Sta_MessageSensors &= ~BOOL6;
			Aux_MMessage += "Ts:";
			Aux_MMessage += Temp_SInterior;
			Aux_MMessage += " ";
		}
		if (Sta_MessageSensors & BOOL7)
		{
			Sta_MessageSensors &= ~BOOL7;
			Aux_MMessage += "Pr:";
			Aux_MMessage += Temp_DewPoint;
			Aux_MMessage += " ";
		}
		if (Sta_MessageSwitch & BOOL1)
		{
			Sta_MessageSwitch &= ~BOOL1;
			Aux_MMessage += "I:";
			if (Sta_Switch & BOOL1)
			{
				Aux_MMessage += "1";
			}
			else
			{
				Aux_MMessage += "0";
			}
			Aux_MMessage += " ";
		}

		if (Sta_MessageSwitch & BOOL2) // Envio mensaje Fan Relay1
		{
			Sta_MessageSwitch &= ~BOOL2;
			Aux_MMessage += "F1:";
			if ((Sta_Compressor & BOOL1) || (Sta_Switch & BOOL3) || (Sta_Switch & BOOL4))
			{
				Aux_MMessage += "1";
			}
			else
			{
				Aux_MMessage += "0";
			}
			Aux_MMessage += " ";
		}

		if (Sta_MessageSwitch & BOOL3) // Envio mensajes velocidad del soplador
		{
			Sta_MessageSwitch &= ~BOOL3;
			Aux_MMessage += "Bl:";
			Aux_MMessage += map(Val_Blower, 0, 255, 0, 6);
			Aux_MMessage += " ";
		}

		if (Sta_MensajeCompressor & BOOL1) // Envio datos del estado del compresor
		{
			Sta_MensajeCompressor &= ~BOOL1;
			Aux_MMessage += "Co:";
			if (Sta_Compressor & BOOL5)
			{
				Aux_MMessage += "1";
			}
			else
			{
				Aux_MMessage += "0";
			}
			Aux_MMessage += " ";
		}

		Aux_MMessage += End_Trama;
		Serial.println(Aux_MMessage);
		Aux_MMessage = " ";
		Sta_Auto &= ~BOOL2;
	}
}