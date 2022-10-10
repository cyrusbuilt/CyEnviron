#include "LDR.h"

#define MAX_DARK_THRESHOLD 1300
#define MAX_DIM_THRESHOLD 800
#define MAX_LIGHT_THRESHOLD 400
#define MAX_BRIGHT_THRESHOLD 100

#define LDR_READING_MAX 1300
#define LDR_READING_MIN 0
#define LDR_READING_OUT_OF_RANGE 30000

LDR::LDR(short pin) {
	_pin = pin;
}

bool LDR::begin() {
	return readSensorRaw() != 0;
}

int LDR::readSensorRaw() {
	int reading = LDR_READING_MIN;

	// Set pin to output and pull low.
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);

	// Switch to input mode and see how long it takes to get to Vcc / 2.
	// This is about 1.65V for the ESP8266.
	pinMode(_pin, INPUT);
	while (digitalRead(_pin) == LOW) {
		reading++;
		if (reading == LDR_READING_OUT_OF_RANGE) {
			break;
		}
	}

	return reading;
}

void LDR::performReading() {
	_rawReading = readSensorRaw();
}

float LDR::readSensorVolts() {
	return _rawReading * (5.0 / 1300.0);
}

int LDR::readSensorBrightness() {
	int inverted = LDR_READING_MAX - _rawReading;
	if (inverted < LDR_READING_MIN) {
		inverted = LDR_READING_MIN;
	}

	if (inverted > LDR_READING_MAX) {
		inverted = LDR_READING_MAX;
	}

	return map(inverted, LDR_READING_MIN, LDR_READING_MAX, 0, 100);
}

BrightnessLevel LDR::getBrightnessLevel() {
	int brightness = _rawReading;
	Serial.print(F("DEBUG: Raw light reading: "));
	Serial.println(brightness);
	if (brightness < MAX_BRIGHT_THRESHOLD) {
		return BrightnessLevel::BRIGHT;
	}
	else if (brightness < MAX_LIGHT_THRESHOLD) {
		return BrightnessLevel::LIGHT;
	}
	else if (brightness < MAX_DIM_THRESHOLD) {
		return BrightnessLevel::DIM;
	}
	else if (brightness < MAX_DARK_THRESHOLD) {
		return BrightnessLevel::DARK;
	}
	else {
		return BrightnessLevel::VERY_DARK;
	}
}