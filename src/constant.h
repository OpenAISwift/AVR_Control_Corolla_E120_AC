#pragma once
#ifndef _CONSTANT_H
#define _CONSTANT_H

/*CONSTANTES COMUNICACION SERIAL*/
#define Ini_Trama '<'   // Caractere para el inicio de trama del mensaje
#define End_Trama '>'   // Caractere para el fin de trama del mensaje
#define Len_BufferInt 30 // Tamaño del buffer de entrada
#define Len_BufferOut 40 // Tamaño del buffer de salida

/*CONSTANTES CONTROL COMPRESOR DEL AIRE ACONDICIONADO*/
#define Val_UmBaj 1  // Valor umbral de temperatura para apagado del embrague magnetico del comprespr del aire acondicionado
#define Val_UmAlt 10 // Valor umbral de temperatura encendido del embrague magnetico del comprespr del aire acondicionado

/*CONSTANTES TERMISTORES Y LECURA DE SENSORES*/
//EVAPORADOR
#define Ae 1.139754081E-3  // Valor modelo de Steinhart-Hart termistor evaporador
#define Be 3.299006989E-4  // Valor modelo de Steinhart-Hart termistor evaporador
#define Ce -4.900267273E-7 // Valor modelo de Steinhart-Hart termistor evaporador

//AMBIENTE
#define Aa 1.076211817E-3  // Valor modelo de Steinhart-Hart termistor ambiente
#define Ba 3.394055633e-4  // Valor modelo de Steinhart-Hart termistor ambiente
#define Ca -5.362332983e-7 // Valor modelo de Steinhart-Hart termistor ambiente

//CONVERSIONES
#define Raux 9800.0 // Valor resistencia para el divisor de voltaje
#define Vcc 4.83    // Alimentacion de Sensores de Temperatura (4,8)
#define K 1.5       // Constante de factor de disipacion en mW/C

/*CONSTANTES TEMPORIZADORES*/
#define Int_Prom 1000 // Intervalo de tiempo lectura sensores analogos
#define Int_LecT 4000 // Intervalo de tiempo lectura sensores digitales
#define Int_MenT 1000 // Intervalo de tiempo envio mensaejes sensores de temperatura
#define Int_Dese 1000 // Intervalo de tiempo espera activavion de desempañador automatico
#define Int_ActE 4000 // Intervalo de tiempo espera activacion de aire acondicionado

#endif
