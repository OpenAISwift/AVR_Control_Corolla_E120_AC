#pragma once
#ifndef _CONSTANT_H
#define _CONSTANT_H

/*CONSTANTES COMUNICACION SERIAL*/
const unsigned long Spe_Serial = 115200; // Baudrate serial port
const uint8_t Len_BufferInt = 30;        // Tamaño del buffer de entrada
const uint8_t Len_BufferOut = 40;        // Tamaño del buffer de salida
const char Ini_Trama = '<';              // Caractere para el inicio de trama del mensaje
const char End_Trama = '>';              // Caractere para el fin de trama del mensaje

/*CONSTANTES CONTROL COMPRESOR DEL AIRE ACONDICIONADO*/
const uint8_t Val_UmBaj = 1;  // Valor umbral de temperatura para apagado del embrague magnetico del comprespr del aire acondicionado
const uint8_t Val_UmAlt = 10; // Valor umbral de temperatura encendido del embrague magnetico del comprespr del aire acondicionado

/*CONSTANTES TERMISTORES Y LECURA DE SENSORES*/
//EVAPORADOR
const float Ae = 1.139754081E-3;  // Valor modelo de Steinhart-Hart termistor evaporador
const float Be = 3.299006989E-4;  // Valor modelo de Steinhart-Hart termistor evaporador
const float Ce = -4.900267273E-7; // Valor modelo de Steinhart-Hart termistor evaporador

//AMBIENTE
const float Aa = 1.076211817E-3;  // Valor modelo de Steinhart-Hart termistor ambiente
const float Ba = 3.394055633e-4;  // Valor modelo de Steinhart-Hart termistor ambiente
const float Ca = -5.362332983e-7; // Valor modelo de Steinhart-Hart termistor ambiente

//CONVERSIONES
const float Raux = 9800.0; // Valor resistencia para el divisor de voltaje
const float Vcc = 4.83;    // Alimentacion de Sensores de Temperatura (4,8)
const float K = 1.5;       // Constante de factor de disipacion en mW/C

/*CONSTANTES TEMPORIZADORES*/
const unsigned int Int_LecTermistores = 1000; // Intervalo de tiempo lectura sensores analogos
const unsigned int Int_LecDigital = 2500;     // Intervalo de tiempo lectura sensores digitales
const unsigned int Int_MenT = 1000;           // Intervalo de tiempo envio mensaejes sensores de temperatura
const unsigned int Int_Dese = 1000;           // Intervalo de tiempo espera activavion de desempañador automatico
const unsigned int Int_ActE = 4000;           // Intervalo de tiempo espera activacion de aire acondicionado

#endif
