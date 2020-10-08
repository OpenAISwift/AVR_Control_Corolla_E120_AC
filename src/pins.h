#pragma once
#ifndef _PINS_H

/*PUERTOS DE ENTRADA ANALOGICOS SENSORES DE TEMPERATURA*/

//TERMISTORES
#define Ter_Evaporador A0 // Termistor Evaporador
#define Ter_Ambient A1    // Termistor Ambiente

//SENSORES DIGITALES
#define Dht_Room 6 // Sensor Temperatura y Humedad Interior

/*SENSORES DE ILUMINACION*/
#define Fot_Solar A2 // Sensor de luz Fototransistor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*PUERTOS DE SALIDA DIGITAL*/

//PUERTOS DE SALIDA CONTROL DE RELAYS
#define Rel_Heater 22       // Heater relay A/C
#define Rel_Cluch 23        // Clutch relay A/C
#define Rel_Defroster 24    // Desempañador Trasero
#define Rel_Fan1 28         // Fan No 1
#define Rel_WarningLight 30 // Señal de advertencia

//PUERTOS DE SALIDA CONTROL DIGITAL
#define Dig_AirFresh 25      // Aire Fresco
#define Dig_Recirculation 26 // Recirculacion
#define Dig_Ac1 27           // A/c Activador

//PUERTOS DE SALIDA CONTROL PWM
#define Pwm_Blower 2 // Ventilador Aire_Acondicionado

//PUERTOS DE SALIDA CONTROL SERVOS
#define Ser_AirMix 9    // Pin Servo mescla aire frio y caliente
#define Ser_VentMode 10 // Pin Servo posairventmode
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*PUERTOS DE ENTRADA DIGITAL*/

//PUERTOS DE ENTRADA DIGITAL INTERRUPTORES
#define Swi_Ill 41         //Interruptor Posicion de Iluminacion
#define Swi_DualAC 43   // Interruptor Dual (preostato)
#define Swi_SingleAC 45 // Interruptor Simple (preostato)
#define Dig_Act 47      // A/C Cortar (ECU)

#endif
