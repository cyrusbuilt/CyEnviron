#ifndef _ENVIRONMENT_H
#define _ENVIRONMENT_H

#include <Arduino.h>
#include "LDR.h"

typedef struct {
	float tempF;
	float tempC;
	double pressureHpa;
	float humidity;
	double gasKohms;
	float altitudeM;
	float altitudeF;
	int brightness;
	BrightnessLevel lightLevel;
	char* alarmCondition;
} env_t;

#endif