#ifndef _ENVIRONMENT_H
#define _ENVIRONMENT_H

#include <Arduino.h>
#include "LDR.h"

// TODO This is the default. We should probably make this configurable though
// for the current location and just fall back to this if invalid or omitted.
#define AVG_PRESSURE_SEALEVEL_HPA (1013.25F)

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
	float iaq;
	float co2Equivalent;
	float breathVoc;
	double dewPoint;
	uint8_t aqi;
	String lastUpdate;
} env_t;

enum class AQI : uint8_t {
	EXCELLENT = 0,
	GOOD = 1,
	LIGHTLY_POLLUTED = 2,
	MODERATELY_POLLUTED = 3,
	HEAVILY_POLLUTED = 4,
	SEVERELY_POLLUTED = 5,
	EXTREME_POLLUTION = 6
};

class EnvUtils {
public:
	static float getAltitude(double pressure, float seaLevel);
	static double getDewPoint(float tempC, float humidity);
	static AQI getAQI(float iaq);
};
#endif