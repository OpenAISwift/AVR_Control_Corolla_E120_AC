#pragma once
#ifndef _PINS_H
#define _PINS_H

/*PUERTOS DE ENTRADA ANALOGICOS SENSORES DE TEMPERATURA*/

//TERMISTORES
const uint8_t Ter_Evaporador = A0; // Termistor Evaporador
const uint8_t Ter_Ambient = A1;    // Termistor Ambiente

//SENSORES DIGITALES
const uint8_t Dht_Room = 6; // Sensor Temperatura y Humedad Interior

/*SENSORES DE ILUMINACION*/
const uint8_t Fot_Solar = A2; // Sensor de luz Fototransistor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*PUERTOS DE SALIDA DIGITAL*/

//PUERTOS DE SALIDA CONTROL DE RELAYS
const uint8_t Rel_Heater = 22;       // Heater relay A/C
const uint8_t Rel_Cluch = 23;        // Clutch relay A/C
const uint8_t Rel_Defroster = 24;    // Desempañador Trasero
const uint8_t Rel_Fan1 = 28;         // Fan No 1
const uint8_t Rel_WarningLight = 30; // Señal de advertencia

//PUERTOS DE SALIDA CONTROL DIGITAL
const uint8_t Dig_AirFresh = 25;      // Aire Fresco
const uint8_t Dig_Recirculation = 26; // Recirculacion
const uint8_t Dig_Ac1 = 27;           // A/c Activador

//PUERTOS DE SALIDA CONTROL PWM
const uint8_t Pwm_Blower = 2; // Ventilador Aire_Acondicionado

//PUERTOS DE SALIDA CONTROL SERVOS
const uint8_t Ser_AirMix = 9;    // Pin Servo mescla aire frio y caliente
const uint8_t Ser_VentMode = 10; // Pin Servo posairventmode
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*PUERTOS DE ENTRADA DIGITAL*/

//PUERTOS DE ENTRADA DIGITAL INTERRUPTORES
const uint8_t Swi_Ill = 41;         //Interruptor Posicion de Iluminacion
const uint8_t Swi_DualAC = 43;   // Interruptor Dual (preostato)
const uint8_t Swi_SingleAC = 45; // Interruptor Simple (preostato)
const uint8_t Dig_Act = 47;      // A/C Cortar (ECU)

#endif
