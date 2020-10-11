#include <Arduino.h>
#include "utility.h"


float Fun_VRefADC()
{
	long result;
	ADMUX = (1 << REFS0) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
	delay(2);
	ADCSRA |= (1 << ADSC);
	while (bit_is_set(ADCSRA, ADSC));
	result = ADCL;
	result |= ADCH << 8;
	result = 1125300L / result;
	return result;
}

double Fun_DewPoint(double Val_TempC, double Val_HumeR)
{
	double RATIO = 373.15 / (273.15 + Val_TempC);
	double RHS = -7.90298 * (RATIO - 1);
	RHS += 5.02808 * log10(RATIO);
	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO))) - 1);
	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1);
	RHS += log10(1013.246);
	double VP = pow(10, RHS - 3) * Val_HumeR;
	double T = log(VP / 0.61078);
	return (241.88 * T) / (17.558 - T);
}