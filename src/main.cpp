/*INICIO DE LIBRERIAS*/
#include <Arduino.h>
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
uint8_t Sta_Sensors;

/*
BOOL1 = Interruptor de Iluminacion Interior
BOOL2 = Heater Relay
BOOL3 = Relay Fan1
BOOL4 = Compresor A/C
*/
uint8_t Sta_Switch;

/*
BOOL1 = Interruptor de la Iluminacion
BOOL2 = Estado relay Fan1
BOOL3 = Estado Compresor A/C
*/
uint8_t Sta_Control;

/*
BOOL1 = Control de desempañamiento automatico
BOOL2 = Control de envio de datos
BOOL3 = Control de recepcion de datos
*/
uint8_t Sta_Auto1;

int8_t Temp_Evaporador = 0; // Variable temperatura Evaporador
int8_t Temp_Ambiente = 0;	// Variable temperatura Ambiente
int8_t Temp_Interior = 0;	// Variable temperatura Interior
int8_t Temp_SInterior = 0;	// Variable  Sensacion temperatura Interior
int8_t Temp_DewPoint = 0;	// Valor de Punto de rocio
uint8_t Humi_Interior = 0;	// Variable humedad Interior
uint8_t Illu_Ambiental = 0; // Valor Fotodiodo
int enteroDesdePc = 0;		// Valor entero comando PC

char receivedChars[Len_BufferInt];	// Vector Caracteres recivios
char tempChars[Len_BufferInt];		// Vector tenporal para almacenar caracteres
char mensajeDesdePc[Len_BufferInt]; // Vector Mensaje PC

uint8_t valorSoplador = 0;
uint8_t valorAirMix = 0;
uint8_t Est_AcAac = 255; // Estado para el control de encendido del compresor del aire acondicionado
uint8_t Est_ADese = 255; // Estado para el desempañamiento automatico parabrisa delantera
uint8_t estadoSingleAc = 0;

unsigned long Tim_Current = 0; // Variable que almacena el tiempo desde que se inicio el sistema
/* FIN DEFINICIONES DE VARIABLES */

/*FUNCIONES PROTOTIPO*/
void lecturaComandosPc();
void lecturaSensores();
void Est_ActMens();
void controlAutomatico();
void Con_Comp();
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
				Sta_Switch &= ~BOOL2;
				Est_AcAac = 0; // Desactiva el compresor del aire acondicionado
				digitalWrite(Rel_Heater, LOW);
			}
		}
		valorSoplador = map(enteroDesdePc, 0, 6, 0, 255);
		analogWrite(Pwm_Blower, valorSoplador);
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
			if (Sta_Auto1 & BOOL1) // Comprueba si esta activado el control automatico
			{
				Est_ADese = 2; // Inicia el desempañamiento automatico
				break;
			}
			else
			{
				Est_ADese = 1; // Inicia el desempañador manual
				break;
			}
			break;
		}
		else
		{
			Est_ADese = 0;
			break;
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
		servoVentMode.write(enteroDesdePc);
		break;

	case 'm': // Control de la mescla de aire frio y aire caliente
		if (enteroDesdePc <= 6 && enteroDesdePc >= 0)
		{
			valorAirMix = map(enteroDesdePc, 0, 6, 15, 135);
			servoAirMix.write(valorAirMix);
		}
		break;

	case 'z': //Control de estado automatico
		if (enteroDesdePc == 1)
		{
			Sta_Auto1 |= BOOL1; // Activamos el funcionamiento automatico
			Est_ADese = 2;		// Inicia el desempañamiento automatico
			break;
		}
		else
		{
			Sta_Auto1 &= ~BOOL1; // Desactivamos el funcionamiento automatico
			Est_ADese = 0;		 // Desactiva el desempañamiento automatico
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
	case 0: // Desactiva el desempañamiento automatico

		Est_AcAac = 0;							// Desactiva el compresor del aire acondicionado
		valorSoplador = map(0, 0, 6, 0, 255);	// Asigna la velocidad del Pwm_Blower
		digitalWrite(Rel_Heater, LOW);			// Desactiva el relay del conjunto del aire acondicionado
		analogWrite(Pwm_Blower, valorSoplador); // Asigna el pwm del Pwm_Blower
		digitalWrite(Dig_Recirculation, HIGH);	// Activa la recirculacion
		digitalWrite(Dig_AirFresh, LOW);		// Desactiva la entrada de aire desde afuera
		Est_ADese = 255;						// Cambia al siguiente estado
		break;

	case 1: // Caso 1 para el inicio de desempañamiento manual

		Est_AcAac = 1;							// Activa el compresor del aire acondicionado
		digitalWrite(Rel_Heater, HIGH);			// Activa el relay del conjunto del aire acondicionado
		valorSoplador = map(3, 0, 6, 0, 255);	// Asigna la velocidad del Pwm_Blower
		analogWrite(Pwm_Blower, valorSoplador); // Asigna el pwm del Pwm_Blower
		servoVentMode.write(0);					// Posiciona la regilla para la parabrisa delantera
		digitalWrite(Dig_Recirculation, LOW);	// Desactiva la recirculacion
		digitalWrite(Dig_AirFresh, HIGH);		// Activa la entrada de aire desde afuera
		Est_ADese = 255;						// Cambia al siguiente estado
		break;

	case 2: // Inicia el desempañador automatico

		Est_CAuto |= BOOL1; // Incia la variable para encendido de climatizador segun el punto de rocio
		Est_ADese = 3;		// Cambia al siguiente estado
		break;

	case 3: // Contorl de encendido y apagado del sistema de aire acondicionado

		if (Est_CAuto & BOOL1) // Control de encedido del climatizador segun el punto de rocio
		{
			if (Temp_Ambiente < Temp_DewPoint) // Si la temperatura ambiente es inferior al punto de rocio
			{
				Est_CAuto &= ~BOOL1;				  // Cambia el control del climatizador para el apagado del climatizador
				Est_AcAac = 1;						  // Activa el compresor del aire acondicionado
				digitalWrite(Rel_Heater, HIGH);		  // Activa el conjunto del climatizador
				valorSoplador = map(3, 0, 6, 0, 255); // Asigna la velocidad de 3 al ventilador
				analogWrite(Pwm_Blower, valorSoplador);
				digitalWrite(Dig_Recirculation, LOW); // Desactiva la recirculacion
				digitalWrite(Dig_AirFresh, HIGH);	  // Activa la entrada de aire desde afuera
				servoVentMode.write(0);				  // Dirige todo el aire al parabrisa
				//valorAirMix = map(3, 0, 6, 15, 135);	// Asigna entre la mescla de aire caliente y frio en 2
				//servoAirMix.write(valorAirMix);
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
				valorSoplador = 0;
				analogWrite(Pwm_Blower, valorSoplador);
				digitalWrite(Rel_Heater, LOW);
				//valorAirMix = map(0, 0, 6, 15, 135);	// Asigna entre la mescla de aire caliente y frio en 0
				//servoAirMix.write(valorAirMix);
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
	valorAirMix = map(0, 0, 6, 15, 135);
	servoAirMix.write(valorAirMix);
}

void loop()
{
	Tim_Current = millis(); // Actualiza el tiempo para los temporizadores
	lecturaSensores();
	lecturaComandosPc();
	controlAutomatico();
	Est_ActMens();
	Con_Comp();
	Con_AutoCli();
}

void Con_Comp() // Funcion para el control del compresor del aire acondicionado
{
	/*
	BOOL1 = Control de compresor segun la temperatura del evaporador
	*/
	static uint8_t Est_Comp = 0;

	/*
	Tiem_AaAC = Variable para almacenar el tiempo de la anterior activacion de aire acondicionado
	*/
	static unsigned long Tie_AaAC = 0;

	switch (Est_AcAac)
	{
	case 0: // Desactivacion del sistema del compresor del aire acondicionado

		Est_AcAac = 255; // Cambia al siguiente estado
		digitalWrite(Dig_Ac1, LOW);
		digitalWrite(Rel_Cluch, LOW);
		Sta_Switch &= ~BOOL3; // Desactiva el relay del ventilador de refigeracion del radiador
		Sta_Switch &= ~BOOL4; // Desactiva el estado del compresor

		if (Sta_Auto1 & BOOL1) // Comprueba si esta activado el control automatico del sistema
		{
			Sta_Auto1 |= BOOL2;	  // Activacion de envio de datos
			Sta_Control |= BOOL2; // Envio de datos de estado FAN1
			Sta_Control |= BOOL3; // Envio de datos de estado Compresor
		}
		break;

	case 1: // Inicia la activacion del compresor del sistema del aire acondicionado

		Est_AcAac = 2;				 // Cambia al siguiente estado
		digitalWrite(Dig_Ac1, HIGH); // Activa la salida de control de la ECU para aumentar el ralentí
		Sta_Switch |= BOOL3;		 // Activa el relay del ventilador de refigeracion del radiador
		Sta_Switch |= BOOL4;		 // Activa el estado del compresor

		if (Sta_Auto1 & BOOL1) // Comprueba si esta activado el control automatico del sistema
		{
			Sta_Auto1 |= BOOL2;	  // Activacion de envio de datos
			Sta_Control |= BOOL2; // Envio de datos de estado FAN1
			Sta_Control |= BOOL3; // Envio de datos de estado Compresor
		}
		Tie_AaAC = Tim_Current; // Actualiza el temporizador
		break;

	case 2: // Espera de tiempo asignado a Tie_AaAC para la espera de encendido y comprobacion del compresor del aire acondicionado
		if (Tim_Current - Tie_AaAC >= Int_ActE)
		{
			Est_Comp |= BOOL1; // Reinicia el estado de encendido del del compresor por temperatura
			Est_AcAac = 3;	   // Cambia al siguiente estado
			break;
		}
		else
		{
			break;
		}

	case 3:
		if (!digitalRead(Swi_DualAC)) // Lectura del interruptor doble del sistema del aire acondicionado (0 = Cerrado 1 = Abierto) Abierdo no debe encender el aire acondicionado
		{
			if (!digitalRead(Swi_SingleAC)) // Lectura del interruptor simple del sistema del aire acondicionado (1 = Cerrado 0 = Abierto) Abierdo no debe encender el aire acondicionado
			{
				if (digitalRead(Dig_Act))
				{ // Lectura de la señal de ECU para desactivacion del compresor del aire acondicionado  (1 = Funcionamineto normal 0 = Apagar compresor)
					if (Est_Comp & BOOL1)
					{ // Control del encendido del compresor del aire acondicionado por umbral de temperatura del evaporador
						if (Temp_Evaporador >= Val_UmAlt)
						{								   // Control de encendido cuando la temperatura umbral es superior o igual a la definida
							Est_Comp &= ~BOOL1;			   // Cambio de estado para la activacion de temperatura por debajo o igual al valor umbral
							digitalWrite(Rel_Cluch, HIGH); // Activa el embrague magnetico del compresor del aire acondicionado
							break;
						}
						break;
					}
					else
					{
						if (Temp_Evaporador <= Val_UmBaj) // Control de apagado cuando la temperatura umbral es inferior o igual a la definida
						{
							Est_Comp |= BOOL1;			  // Cambio de estado para la activacion de temperatura superior o igual al valor umbral
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

		digitalWrite(Rel_Cluch, LOW); // Desactiva el embrague magnetico del compresor del aire acondicionado
		Tie_AaAC = Tim_Current;		  // Actualiza el temporizador
		Est_AcAac = 2;				  // Cambia al siguiente estado
		break;

	default: // Caso por defecto no haga nada
		break;
	}
}
void controlAutomatico()
{
	posicionLuz.update();
	if (posicionLuz.read())
	{
		if (!(Sta_Switch & BOOL1))
		{
			Sta_Switch |= BOOL1;
			Sta_Control |= BOOL1;
			Sta_Auto1 |= BOOL2;
		}
	}
	else
	{
		if (Sta_Switch & BOOL1)
		{
			Sta_Switch &= ~BOOL1;
			Sta_Control |= BOOL1;
			Sta_Auto1 |= BOOL2;
		}
	}

	estadoSingleAc = digitalRead(Swi_SingleAC); //Asignacion de estado preostato interruptor simple
	digitalWrite(Rel_Fan1, (estadoSingleAc || (Sta_Switch & BOOL3)));
}
void lecturaComandosPc()
{
	static boolean recvInProgress = false;
	static uint8_t ndx = 0;
	char rc;
	if (Serial.available() > 0 && !(Sta_Auto1 & BOOL3))
	{
		rc = Serial.read();
		if (recvInProgress)
		{
			if (rc != End_Trama)
			{
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= Len_BufferInt)
				{
					ndx = Len_BufferInt - 1;
				}
			}
			else
			{
				receivedChars[ndx] = '\0'; // Termina la cadena
				recvInProgress = false;
				ndx = 0;
				Sta_Auto1 |= BOOL3;
			}
		}
		else if (rc == Ini_Trama)
		{
			recvInProgress = true;
		}
	}
	if (Sta_Auto1 & BOOL3)
	{
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
		Sta_Auto1 &= ~BOOL3;
	}
}
void lecturaSensores()
{
	static unsigned long Tim_PreTermistores = 0; // Tiempo anterior lectura temperatura sensores analogos
	static unsigned long Tim_PreDigital = 0;	 // Tiempo anterior lectura temperatura sensores digitales

	if (Tim_Current - Tim_PreTermistores >= Int_LecTermistores)
	{
		static int8_t Temp_AntEvaporador = 0;											 // Variable temperatura Anterior Evaporador
		static int8_t Temp_AntAmbiente = 0;												 // Variable temperatura Anterior Ambiente
		static uint8_t Illu_AntAmbiental = 0;											 // Valor Fotodiodo
		Temp_AntEvaporador = Fun_ConTemperature(analogRead(Ter_Evaporador), Ae, Be, Ce); // Convercion del voltage a temperatura del evaporador
		Temp_AntAmbiente = Fun_ConTemperature(analogRead(Ter_Ambient), Aa, Ba, Ca);		 // Convercion del voltage a temperatura ambiente
		Illu_AntAmbiental = map(analogRead(Fot_Solar), 0, 1023, 0, 100);				 // Convercion del voltage a iluminacion ambiental

		if (Temp_AntEvaporador != Temp_Evaporador)
		{
			Temp_Evaporador = Temp_AntEvaporador;
			Sta_Auto1 |= BOOL2;
			Sta_Sensors |= BOOL1;
		}
		if (Temp_AntAmbiente != Temp_Ambiente)
		{
			Temp_Ambiente = Temp_AntAmbiente;
			Sta_Auto1 |= BOOL2;
			Sta_Sensors |= BOOL2;
		}
		if (Illu_AntAmbiental != Illu_Ambiental)
		{
			Illu_Ambiental = Illu_AntAmbiental;
			Sta_Auto1 |= BOOL2;
			Sta_Sensors |= BOOL3;
		}

		Tim_PreTermistores = Tim_Current; // Actualiza el tiempo para la siguiente lectura
	}

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
			Sta_Auto1 |= BOOL2;
			Sta_Sensors |= BOOL4;
		}
		if (Humi_AntInterior != Humi_Interior)
		{
			Humi_Interior = Humi_AntInterior;
			Sta_Auto1 |= BOOL2;
			Sta_Sensors |= BOOL5;
		}
		if (Temp_AntSInterior != Temp_SInterior)
		{
			Temp_SInterior = Temp_AntSInterior;
			Sta_Auto1 |= BOOL2;
			Sta_Sensors |= BOOL6;
		}
		if (Temp_AntDewPoint != Temp_DewPoint)
		{
			Temp_DewPoint = Temp_AntDewPoint;
			Sta_Auto1 |= BOOL2;
			Sta_Sensors |= BOOL7;
		}

		Tim_PreDigital = Tim_Current;
	}
}
void Est_ActMens() // Funcion para enviar los datos de los sensores
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
	*/

	String Aux_MMessage;
	if (Sta_Auto1 & BOOL2)
	{
		Aux_MMessage += Ini_Trama;
		Aux_MMessage += " ";
		if (Sta_Sensors & BOOL1)
		{
			Sta_Sensors &= ~BOOL1;
			Aux_MMessage += "Te:";
			Aux_MMessage += Temp_Evaporador;
			Aux_MMessage += " ";
		}
		if (Sta_Sensors & BOOL2)
		{
			Sta_Sensors &= ~BOOL2;
			Aux_MMessage += "Ta:";
			Aux_MMessage += Temp_Ambiente;
			Aux_MMessage += " ";
		}
		if (Sta_Sensors & BOOL3)
		{
			Sta_Sensors &= ~BOOL3;
			Aux_MMessage += "La:";
			Aux_MMessage += Illu_Ambiental;
			Aux_MMessage += " ";
		}
		if (Sta_Sensors & BOOL4)
		{
			Sta_Sensors &= ~BOOL4;
			Aux_MMessage += "Ti:";
			Aux_MMessage += Temp_Interior;
			Aux_MMessage += " ";
		}
		if (Sta_Sensors & BOOL5)
		{
			Sta_Sensors &= ~BOOL5;
			Aux_MMessage += "H:";
			Aux_MMessage += Humi_Interior;
			Aux_MMessage += " ";
		}
		if (Sta_Sensors & BOOL6)
		{
			Sta_Sensors &= ~BOOL6;
			Aux_MMessage += "Ts:";
			Aux_MMessage += Temp_SInterior;
			Aux_MMessage += " ";
		}
		if (Sta_Sensors & BOOL7)
		{
			Sta_Sensors &= ~BOOL7;
			Aux_MMessage += "Pr:";
			Aux_MMessage += Temp_DewPoint;
			Aux_MMessage += " ";
		}
		if (Sta_Control & BOOL1)
		{
			Sta_Control &= ~BOOL1;
			Aux_MMessage += "I:";
			Aux_MMessage += (Sta_Switch & BOOL1);
			Aux_MMessage += " ";
		}
		if (Sta_Control & BOOL2)
		{
			Sta_Control &= ~BOOL2;
			Aux_MMessage += "F1:";
			Aux_MMessage += (Sta_Switch & BOOL3);
			Aux_MMessage += " ";
		}
		if (Sta_Control & BOOL3)
		{
			Sta_Control &= ~BOOL3;
			Aux_MMessage += "Co:";
			Aux_MMessage += (Sta_Switch & BOOL4);
			Aux_MMessage += " ";
		}

		Aux_MMessage += End_Trama;
		Serial.println(Aux_MMessage);
		Aux_MMessage = " ";
		Sta_Auto1 &= ~BOOL2;
	}
}