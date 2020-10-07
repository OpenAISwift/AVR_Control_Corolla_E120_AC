/*INICIO DE LIBRERIAS*/
#include <Arduino.h>
#include "cactus_io_DHT22.h"
#include "Bounce2.h"
#include "Servo.h"
#include <math.h>
/*FIN DE LIBRERIAS*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// 	INICIO DEFINICIONES DE CONFIGURACION DEL SISTEMA																////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define Val_UmBaj 1  // Valor umbral de temperatura para apagado del embrague magnetico del comprespr del aire acondicionado
#define Val_UmAlt 10 // Valor umbral de temperatura encendido del embrague magnetico del comprespr del aire acondicionado
//////////////////////////////////////////////////////////////////////////
#define Ini_Trama '<' // Caractere para el inicio de trama del mensaje
#define End_Trama '>' // Caractere para el fin de trama del mensaje
//////////////////////////////////////////////////////////////////////////
#define Ae 1.139754081E-3  // Valor modelo de Steinhart-Hart termistor evaporador
#define Be 3.299006989E-4  // Valor modelo de Steinhart-Hart termistor evaporador
#define Ce -4.900267273E-7 // Valor modelo de Steinhart-Hart termistor evaporador
//////////////////////////////////////////////////////////////////////////
#define Aa 1.076211817E-3  // Valor modelo de Steinhart-Hart termistor ambiente
#define Ba 3.394055633e-4  // Valor modelo de Steinhart-Hart termistor ambiente
#define Ca -5.362332983e-7 // Valor modelo de Steinhart-Hart termistor ambiente
//////////////////////////////////////////////////////////////////////////
#define Raux 9800.0 // Valor resistencia para el divisor de voltaje
#define Vcc 4.83	// Alimentacion de Sensores de Temperatura (4,8)
#define K 1.5		// Constante de factor de disipacion en mW/C
//////////////////////////////////////////////////////////////////////////
#define Int_Prom 1000 // Intervalo de tiempo lectura sensores analogos
#define Int_LecT 4000 // Intervalo de tiempo lectura sensores digitales
#define Int_MenT 1000 // Intervalo de tiempo envio mensaejes sensores de temperatura
#define Int_Dese 1000 // Intervalo de tiempo espera activavion de desempañador automatico
#define Int_ActE 4000 // Intervalo de tiempo espera activacion de aire acondicionado

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// 	FIN DEFINICIONES DE CONFIGURACION DEL SISTEMA																	////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* INICIO DEFINICION DE PINES */
#define Sen_Eva A0			  // Thermistor Evaporador
#define sensorAmbiente A1	 // Thermistor Ambiente
#define sensorluzAmbiental A2 // Fotodiodo
#define sensorIterior 6		  // Sensor Temperatura Interior
#define coolantTemperature 29 // Opcional (Averiguar tipo de Señal)

DHT22 dht(sensorIterior);

#define pinPosicionLuz 41 // Interruptor Posicion de Iluminacion
Bounce posicionLuz = Bounce();
#define dualAC 43   // Interruptor Dual (preostato)
#define singleAC 45 // Interruptor Simple (preostato)

#define pinAirMix 9 // Pin Servo mescla aire frio y caliente
Servo servoAirMix;
#define pinVentMode 10 // Pin Servo posairventmode
Servo servoVentMode;
#define positionAirFresh 25		 // Aire Fresco
#define positionRecirculation 26 // Recirculacion

#define soplador 2		  // Ventilador Aire_Acondicionado
#define heaterRelay 22	// Heater relay
#define cluchRelay 23	 // Clutch relay a/c
#define defrosterRelay 24 // Desempañador Trasero
#define activatorAC 27	// A/c Activador
#define relayFan1 28	  // Relay Fan No 1
#define warningLight 30   // Señal de peligro 30
#define ACT 47			  // A/C Cortar (ECU)
/* FIN DEFINICION DE PINES */

/* INICO DEFINICIONES DE VARIABLES */
constexpr byte
	numeroDeCaracteres = 32;
float
	ADCvRef = 0;
int
	tempEvaporador = 0,	// Variable temperatura Evaporador
	tempAmbiente = 0,	  // Variable temperatura Ambiente
	tempInterior = 0,	  // Variable temperatura Interior
	tempSensaInterior = 0, // Variable  Sensacion temperatura Interior
	humInterior = 0,	   // Variable humedad Interior
	puntoRocio = 0,		   // Valor de Punto de rocio
	valorLuzAmbiental = 0, // Valor Fotodiodo
	enteroDesdePc = 0;	 // Valor entero comando PC

char
	receivedChars[numeroDeCaracteres],  // Vector Caracteres recivios
	tempChars[numeroDeCaracteres],		// Vector tenporal para almacenar caracteres
	mensajeDesdePc[numeroDeCaracteres], // Vector Mensaje PC
	buffer[60];

uint8_t
	Est_Auto1 = 0, // Estado boleanos para el control automaico

	/*
	BOOL1 = Estado boleano para el control de desempañamiento automatico
	*/

	valorSoplador = 0,
	valorAirMix = 0,
	Est_AcAac = 255, // Estado para el control de encendido del compresor del aire acondicionado
	Est_ADese = 255, // Estado para el desempañamiento automatico parabrisa delantera
	Est_RFan1 = 0,   // Estado control del relay de la refrigeracion del motor
	Est_MAuto = 0,   // Estado para el envio de mensajes de sensores de temperatura
	estadoCuartos = 0,
	flagCuartos = 0,
	estadoHeaterRelay = 0,
	estadoSingleAc = 0,
	mensajeNuevo = 0;

unsigned long
	Tie_Actu = 0; // Variable que almacena el tiempo desde que se inicio el sistema
/* FIN DEFINICIONES DE VARIABLES */

/* INICIO DE MACROS */
#define BOOL1 0x01
#define BOOL2 0x02
#define BOOL3 0x04
#define BOOL4 0x08
#define BOOL5 0x10
#define BOOL6 0x20
#define BOOL7 0x40
#define BOOL8 0x80
/*
Para comprobar un estado:
if ( "nombre de variable" & BOOLX ){
// Ejecuta alguna instrucción
}
Establece el estado 1
"nombre de variable" |= BOOLX;
Limpia el estado BOOL1
"nombre de variable" &=~ BOOLX;
*/
/* FIN DE MACROS */

float vRefADC()
{
	long result;
	ADMUX = (1 << REFS0) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
	delay(2);
	ADCSRA |= (1 << ADSC);
	while (bit_is_set(ADCSRA, ADSC))
		;
	result = ADCL;
	result |= ADCH << 8;
	result = 1125300L / result;
	return result;
}

double lecturaPuntoRocio(double temperaturaC, double humedadR)
{
	double RATIO = 373.15 / (273.15 + temperaturaC);
	double RHS = -7.90298 * (RATIO - 1);
	RHS += 5.02808 * log10(RATIO);
	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO))) - 1);
	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1);
	RHS += log10(1013.246);
	double VP = pow(10, RHS - 3) * humedadR;
	double T = log(VP / 0.61078);
	return (241.88 * T) / (17.558 - T);
}
float temperatura(int RawADC, float A, float B, float C)
{
	float Vo = ((float)RawADC + 0.5) / 1024.0 * ADCvRef;
	float resTermistor = (ADCvRef * Raux / Vo) - Raux;
	float logRes = log(resTermistor);
	float rTe = 1 / (A + (B * logRes) + (C * logRes * logRes * logRes));
	float temC = rTe - 273.15;
	//temC = temC - (Vo * Vo) / (K * resTermistor); // Se desconoce el valor de dicipacion
	return temC;
}

void lecturaSensores()
{
	static int
		ADC_Acum = 0; // Variable para acumular las lecturas del ADC
	static unsigned long
		Tie_ATeA = 0, // Tiempo anterior lectura temperatura sensores analogos
		Tie_ATeD = 0, // Tiempo anterior lectura temperatura sensores digitales
		ADC_LuzA = 0, // Variable para acumular las lecturas del sensor de Luz
		ADC_TemE = 0, // Variable para acumular las lecturas del sensor de temperatura Evaporador
		ADC_TemA = 0; // Variable para acumular las lecturas del sensor de temperatura Ambiente

	ADC_LuzA += analogRead(sensorluzAmbiental); // Acumula las lecturas del sensor de luz ambiente
	ADC_TemA += analogRead(sensorAmbiente);		// Acumula las lecturas del sensor de temperatura Ambiente
	ADC_TemE += analogRead(Sen_Eva);			// Acumula las lecturas del sensor de temperatura del evaporador
	ADC_Acum++;									// Acumulador para el numero de lecturas de los sensores

	estadoSingleAc = digitalRead(singleAC); //Asignacion de estado preostato interruptor simple
	posicionLuz.update();					// Actualiza el estados de las luces
	estadoCuartos = posicionLuz.read();		//Asignacion de estado luces

	if (Tie_Actu - Tie_ATeA >= Int_Prom)
	{
		unsigned int
			ADC_ProA = ADC_TemA / ADC_Acum,					// Variable temporal para almacenar el promedio de las lecturas de sensores
			ADC_ProE = ADC_TemE / ADC_Acum,					// Variable temporal para almacenar el promedio de las lecturas de sensores
			ADC_ProL = ADC_LuzA / ADC_Acum;					// Variable temporal para almacenar el promedio de las lecturas de sensores
		tempAmbiente = temperatura(ADC_ProA, Aa, Ba, Ca);   // Convercion del voltage a temperatura ambiente
		tempEvaporador = temperatura(ADC_ProE, Ae, Be, Ce); // Convercion del voltage a temperatura del evaporador
		valorLuzAmbiental = map(ADC_ProL, 0, 1023, 0, 100);
		ADC_Acum = 0;		 // Reinicia las variables para las siguientes lecturas
		ADC_TemA = 0;		 // Reinicia las variables para las siguientes lecturas
		ADC_TemE = 0;		 // Reinicia las variables para las siguientes lecturas
		ADC_LuzA = 0;		 // Reinicia las variables para las siguientes lecturas
		Tie_ATeA = Tie_Actu; // Actualiza el tiempo para la siguiente lectura
		Est_MAuto = 1;		 // Estado para el control de mensajes
	}
	if (Tie_Actu - Tie_ATeD >= Int_LecT)
	{
		dht.readTemperature();
		dht.readHumidity();
		tempInterior = dht.temperature_C;
		humInterior = dht.humidity;
		tempSensaInterior = dht.computeHeatIndex_C();
		puntoRocio = lecturaPuntoRocio(tempInterior, humInterior);
		Tie_ATeD = Tie_Actu; // Actualiza el tiempo para la siguiente lectura
		Est_MAuto = 2;		 // Estado para el control de mensajes
	}
}
void controlAutomatico()
{
	if (estadoCuartos)
	{
		if (!flagCuartos)
		{
			flagCuartos = 1;
			sprintf(buffer, "< I:%d >", flagCuartos);
			Serial.println(buffer);
		}
	}
	else
	{
		if (flagCuartos)
		{
			flagCuartos = 0;
			sprintf(buffer, "< I:%d >", flagCuartos);
			Serial.println(buffer);
		}
	}
}
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
	<z, "valor">->Desempañador automatico parabrisa delantera (1 = Encendido, 0 = Apagado)
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

	case 'b': // Control del soplador del sistema del aire acondicionado
		if (enteroDesdePc >= 1)
		{
			if (!estadoHeaterRelay)
			{
				estadoHeaterRelay = 1;
				digitalWrite(heaterRelay, HIGH);
			}
		}
		else
		{
			if (estadoHeaterRelay)
			{
				estadoHeaterRelay = 0;
				Est_AcAac = 0; // Desactiva el compresor del aire acondicionado
				digitalWrite(heaterRelay, LOW);
			}
		}
		valorSoplador = map(enteroDesdePc, 0, 6, 0, 255);
		analogWrite(soplador, valorSoplador);
		break;

	case 'c': // Control del sistema de refigeracion del motor
		if (enteroDesdePc == 1)
		{
			Est_RFan1 = 1;
			break;
		}
		else
		{
			Est_RFan1 = 0;
			break;
		}
		break;

	case 'd': // Control del desempañamiento de la parabrisa trasera
		if (enteroDesdePc == 1)
		{
			digitalWrite(defrosterRelay, HIGH);
			break;
		}
		else
		{
			digitalWrite(defrosterRelay, LOW);
			break;
		}
		break;

	case 'e': // Control del desempañador Parabrisas delantero
		if (enteroDesdePc == 1)
		{
			if (Est_Auto1 & BOOL1) // Comprueba si esta activado el control automatico
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
		break;

	case 'f': // Control de recirculacion del sistema del aire acondicionado
		if (enteroDesdePc == 1)
		{
			digitalWrite(positionRecirculation, HIGH);
			digitalWrite(positionAirFresh, LOW);
			break;
		}
		else
		{
			digitalWrite(positionRecirculation, LOW);
			digitalWrite(positionAirFresh, HIGH);
			break;
		}
		break;

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

	case 'z':					// Desempañador Automatico parabrisas delantera
		if (enteroDesdePc == 1) // Cuando este en 1 el desempañador automatico estara funcionando
		{
			Est_Auto1 |= BOOL1; // Activamos el funcionamiento del desempañador automatico
			break;
		}
		else
		{
			Est_Auto1 &= ~BOOL1; // Desactivamos el funcionamiento del desempañador automatico
			break;
		}
		break;

	case 'w': // Encender luces de emergencia
		if (enteroDesdePc == 1)
		{
			digitalWrite(warningLight, HIGH);
			break;
		}
		else
		{
			digitalWrite(warningLight, LOW);
			break;
		}
		break;

	default: // Caso por defecto no haga nada
		break;
	}
}
void lecturaComandosPc()
{
	static boolean recvInProgress = false;
	static byte ndx = 0;
	char rc;
	if (Serial.available() > 0 && !mensajeNuevo)
	{
		rc = Serial.read();
		if (recvInProgress)
		{
			if (rc != End_Trama)
			{
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= numeroDeCaracteres)
				{
					ndx = numeroDeCaracteres - 1;
				}
			}
			else
			{
				receivedChars[ndx] = '\0'; // Termina la cadena
				recvInProgress = false;
				ndx = 0;
				mensajeNuevo = true;
			}
		}
		else if (rc == Ini_Trama)
		{
			recvInProgress = true;
		}
	}
	if (mensajeNuevo)
	{
		strcpy(tempChars, receivedChars);
		/* Divide los datos en sus partes*/
		/////////////////////////////////////////////////////////////////////////////////////////////////
		char *strtokIndx;					 // Es utilizado por strtok() como un indice
		strtokIndx = strtok(tempChars, ","); // Optiene la primera parte de la cadena
		strcpy(mensajeDesdePc, strtokIndx);  // Copia en mensajeDesdePc
		strtokIndx = strtok(NULL, ",");		 // Contionua donde quedo la llamada anterior
		enteroDesdePc = atoi(strtokIndx);	// Convierte a un numero entero
		/////////////////////////////////////////////////////////////////////////////////////////////////
		controlManual();
		mensajeNuevo = false;
	}
}
void Con_EstComp() // Funcion para el control del compresor del aire acondicionado
{
	static uint8_t
		Est_Comp = 0;

	/*
	BOOL1 = Control de compresor segun la temperatura del evaporador
	*/

	static unsigned long
		Tie_AaAC = 0; // Variable para almacenar el tiempo de la anterior activacion de aire acondicionado

	/*
	Tiem_AaAC = Variable para almacenar el tiempo de la anterior activacion de aire acondicionado
	*/

	switch (Est_AcAac)
	{
	case 0: // Desactivacion del sistema del compresor del aire acondicionado

		Est_AcAac = 255;				// Cambia al siguiente estado
		digitalWrite(activatorAC, LOW); // Desactiva la salida para la ECU aumente el ralentí
		digitalWrite(cluchRelay, LOW);  // Desactiva el embrague magnetico del compresor del aire acondicionado
		Est_RFan1 = 0;					// Desactiva el relay del ventilador de refigeracion del radiador
		break;

	case 1: // Inicia la activacion del compresor del sistema del aire acondicionado

		Est_AcAac = 2;					 // Cambia al siguiente estado
		digitalWrite(activatorAC, HIGH); // Activa la salida para la ECU aumente el ralentí
		Tie_AaAC = Tie_Actu;			 // Actualiza el temporizador
		Est_RFan1 = 1;					 // Activa el relay del ventilador de refigeracion del radiador
		break;

	case 2: // Espera de tiempo asignado a Tie_AaAC para la espera de encendido y comprobacion del compresor del aire acondicionado
		if (Tie_Actu - Tie_AaAC >= Int_ActE)
		{
			Est_Comp |= BOOL1; // Reinicia el estado de encendido del del compresor por temperatura
			Est_AcAac = 3;	 // Cambia al siguiente estado
			break;
		}
		else
		{
			break;
		}
		break;

	case 3:
		if (!digitalRead(dualAC)) // Lectura del interruptor doble del sistema del aire acondicionado (0 = Cerrado 1 = Abierto) Abierdo no debe encender el aire acondicionado
		{
			if (!digitalRead(singleAC)) // Lectura del interruptor simple del sistema del aire acondicionado (1 = Cerrado 0 = Abierto) Abierdo no debe encender el aire acondicionado
			{
				if (digitalRead(ACT))
				{ // Lectura de la señal de ECU para desactivacion del compresor del aire acondicionado  (1 = Funcionamineto normal 0 = Apagar compresor)
					if (Est_Comp & BOOL1)
					{ // Control del encendido del compresor del aire acondicionado por umbral de temperatura del evaporador
						if (tempEvaporador >= Val_UmAlt)
						{									// Control de encendido cuando la temperatura umbral es superior o igual a la definida
							Est_Comp &= ~BOOL1;				// Cambio de estado para la activacion de temperatura por debajo o igual al valor umbral
							digitalWrite(cluchRelay, HIGH); // Activa el embrague magnetico del compresor del aire acondicionado
							break;
						}
						break;
					}
					else
					{
						if (tempEvaporador <= Val_UmBaj) // Control de apagado cuando la temperatura umbral es inferior o igual a la definida
						{
							Est_Comp |= BOOL1;			   // Cambio de estado para la activacion de temperatura superior o igual al valor umbral
							digitalWrite(cluchRelay, LOW); // Desactiva el embrague magnetico del compresor del aire acondicionado
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

		digitalWrite(cluchRelay, LOW); // Desactiva el embrague magnetico del compresor del aire acondicionado
		Tie_AaAC = Tie_Actu;		   // Actualiza el temporizador
		Est_AcAac = 2;				   // Cambia al siguiente estado
		break;

	default: // Caso por defecto no haga nada
		break;
	}
}
void Con_AutoCli() // Funcion para el control automatico del climatizador
{
	static uint8_t
		Est_CAuto = 0; // Control de encendido climatizcion
	/* 
	BOOl1 = Encendido o apagado segun el punto de rocio calculado 
	*/

	switch (Est_ADese)
	{
	case 0: // Desactiva el desempañamiento automatico

		Est_AcAac = 0;							   // Desactiva el compresor del aire acondicionado
		valorSoplador = map(0, 0, 6, 0, 255);	  // Asigna la velocidad del soplador
		digitalWrite(heaterRelay, LOW);			   // Desactiva el relay del conjunto del aire acondicionado
		analogWrite(soplador, valorSoplador);	  // Asigna el pwm del soplador
		digitalWrite(positionRecirculation, HIGH); // Activa la recirculacion
		digitalWrite(positionAirFresh, LOW);	   // Desactiva la entrada de aire desde afuera
		Est_ADese = 255;						   // Cambia al siguiente estado
		break;

	case 1: // Caso 1 para el inicio de desempañamiento manual

		Est_AcAac = 1;							  // Activa el compresor del aire acondicionado
		valorSoplador = map(3, 0, 6, 0, 255);	 // Asigna la velocidad del soplador
		digitalWrite(heaterRelay, HIGH);		  // Activa el relay del conjunto del aire acondicionado
		analogWrite(soplador, valorSoplador);	 // Asigna el pwm del soplador
		servoVentMode.write(0);					  // Posiciona la regilla para la parabrisa delantera
		digitalWrite(positionRecirculation, LOW); // Desactiva la recirculacion
		digitalWrite(positionAirFresh, HIGH);	 // Activa la entrada de aire desde afuera
		Est_ADese = 255;						  // Cambia al siguiente estado
		break;

	case 2: // Inicia el desempañador automatico

		Est_CAuto |= BOOL1; // Incia la variable para encendido de climatizador segun el punto de rocio

		Est_ADese = 3; // Cambia al siguiente estado
		break;

	case 3: // Contorl de encendido y apagado del sistema de aire acondicionado

		if (Est_CAuto & BOOL1) // Control de encedido del climatizador segun el punto de rocio
		{
			if (tempAmbiente < puntoRocio) // Si la temperatura ambiente es inferior al punto de rocio
			{
				Est_CAuto &= ~BOOL1;			 // Cambia el control del climatizador para el apagado del climatizador
				Est_AcAac = 1;					 // Activa el compresor del aire acondicionado
				digitalWrite(heaterRelay, HIGH); // Activa el conjunto del climatizador
				analogWrite(soplador, valorSoplador);
				valorSoplador = map(3, 0, 6, 0, 255);	 // Asigna la velocidad de 3 al ventilador
				digitalWrite(positionRecirculation, LOW); // Desactiva la recirculacion
				digitalWrite(positionAirFresh, HIGH);	 // Activa la entrada de aire desde afuera
				servoVentMode.write(0);					  // Dirige todo el aire al parabrisa
				//valorAirMix = map(3, 0, 6, 15, 135);	// Asigna entre la mescla de aire caliente y frio en 2
				//servoAirMix.write(valorAirMix);
				break;
			}
			break;
		}
		else
		{
			if (tempAmbiente > puntoRocio + 3)
			{
				Est_CAuto |= BOOL1; // Cambia el control del climatizador para el encendido del climatizador
				Est_AcAac = 0;		// Desactiva el compresor del aire acondicionado
				valorSoplador = 0;
				analogWrite(soplador, valorSoplador);
				digitalWrite(heaterRelay, LOW);
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
void Est_ActMens()
{ // Funcion para enviar los datos de los sensores
	switch (Est_MAuto)
	{
	case 1: // Envia mensaje de sensores analogicos
		sprintf(buffer, "< Ta:%d,Te:%d,La:%d >", tempAmbiente, tempEvaporador, valorLuzAmbiental);
		Serial.println(buffer);
		Est_MAuto = 0;
		break;

	case 2: // Envia mensaje de sensores digitales
		sprintf(buffer, "< Ti:%d,H:%d,Ts:%d,Pr:%d,Ta:%d,Te:%d,La:%d >", tempInterior, humInterior, tempSensaInterior, puntoRocio, tempAmbiente, tempEvaporador, valorLuzAmbiental);
		Serial.println(buffer);
		Est_MAuto = 0;
		break;

	default: // Caso por defecto no haga nada
		break;
	}
}

void setup()
{
	Serial.begin(115200UL);
	dht.begin();

	pinMode(soplador, OUTPUT);
	pinMode(heaterRelay, OUTPUT);
	pinMode(defrosterRelay, OUTPUT);
	pinMode(positionRecirculation, OUTPUT);
	pinMode(positionAirFresh, OUTPUT);
	pinMode(cluchRelay, OUTPUT);
	pinMode(relayFan1, OUTPUT);
	pinMode(activatorAC, OUTPUT);
	pinMode(warningLight, OUTPUT);

	pinMode(coolantTemperature, INPUT);
	pinMode(pinPosicionLuz, INPUT);
	pinMode(singleAC, INPUT);
	pinMode(dualAC, INPUT);
	pinMode(ACT, INPUT);
	ADCvRef = vRefADC() / 1000.0;

	servoAirMix.attach(pinAirMix);
	servoVentMode.attach(pinVentMode);
	posicionLuz.attach(pinPosicionLuz);
	posicionLuz.interval(5);
	valorAirMix = map(0, 0, 6, 15, 135);
	servoAirMix.write(valorAirMix);
}

void loop()
{
	Tie_Actu = millis(); // Actualiza el tiempo para los temporizadores
	digitalWrite(relayFan1, (estadoSingleAc || Est_RFan1));
	lecturaSensores();
	lecturaComandosPc();
	controlAutomatico();
	Est_ActMens();
	Con_EstComp();
	Con_AutoCli();
}