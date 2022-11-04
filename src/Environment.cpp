#include "Environment.h"

float EnvUtils::getAltitude(double pressure, float seaLevel) {
	float atmospheric = pressure / 100.0F;
	return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

double EnvUtils::getDewPoint(float tempC, float humidity) {
	const double a = 17.271;
	const double b = 237.7;
	double temp = (a * tempC) / (b + tempC) + log(humidity * 0.01);
	return (b * temp) / (a - temp);
}

double EnvUtils::convertTempCtoF(double celcius) {
	return celcius * 9 / 5 + 32;
}

AQI EnvUtils::getAQI(float iaq) {
	if (iaq <= 50) {
		return AQI::EXCELLENT;
	}
	else if (iaq > 50 && iaq <= 100) {
		return AQI::GOOD;
	}
	else if (iaq > 100 && iaq <= 150) {
		return AQI::LIGHTLY_POLLUTED;
	}
	else if (iaq > 150 && iaq <= 200) {
		return AQI::MODERATELY_POLLUTED;
	}
	else if (iaq > 200 && iaq <= 250) {
		return AQI::HEAVILY_POLLUTED;
	}
	else if (iaq > 250 && iaq <= 350) {
		return AQI::SEVERELY_POLLUTED;
	}
	else {
		return AQI::EXTREME_POLLUTION; 
	}
}