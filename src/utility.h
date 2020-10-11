#pragma once
#ifndef _UTILITY_H
#define _UTILITY_H

void Ini_FunParameter();
float Fun_VRefADC(); // Funcion para la lectura del voltaje de alimentacion del ADC de AVR
double Fun_DewPoint(double Val_TempC, double Val_HumeR); // Punto de rocio
float Fun_ConTemperature(int RawADC, float A, float B, float C);

#endif