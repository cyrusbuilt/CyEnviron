#ifndef _LDR_H
#define _LDR_H

#include <Arduino.h>

/**
 * @brief Possible light levels.
 */
enum class BrightnessLevel : uint8_t {
	DARK = 0,
	DIM = 1,
	LIGHT = 2,
	BRIGHT = 3,
	VERY_DARK = 4
};

/**
 * @brief Driver for LDR/CDS Photocell sensors. Unlike standard light sensors
 * though, this implementation doesn't use the ADC pin on the MCU. Instead,
 * it uses any digital I/O pin combined with the sensor and a 0.1uF capacitor
 * and this driver measures the time it takes for the capacitor to charge to
 * Vcc / 2 (approx. 1.65V on the ESP8266).
 */
class LDR {
public:
	/**
	 * @brief Construct a new LDR object.
	 * 
	 * @param pin The pin to assign to the LDR/CDS sensor.
	 */
	LDR(short pin);

	/**
	 * @brief Initializes the driver.
	 * 
	 * @return true if the LDR sensor was detected.
	 * @return false if the LDR sesnor wasn't found.
	 */
	bool begin();

	/**
	 * @brief Gets the current raw sensor reading and stores it in memory. This
	 * should either be called from your loop() method with a delay (rougly 10ms)
	 * between readings or from a recurring task with adequate delay to allow the
	 * sensor circuit to settle.
	 */
	void performReading();

	/**
	 * @brief Reads the raw sensor value. While this can be called directly,
	 * it is recommended you call performReading() instead followed by
	 * readSensorBrightness() or getBrightnessLevel().
	 * 
	 * @return int The sensor reading the time (in cycles) it took to capacitor
	 * to reach Vcc / 2.
	 */
	int readSensorRaw();

	/**
	 * @brief Gets the sensor reading in volts.
	 * 
	 * @return float The current voltage.
	 */
	float readSensorVolts();

	/**
	 * @brief Gets brightness value.
	 * 
	 * @return int Brightness percentage (0 - 100).
	 */
	int readSensorBrightness();

	/**
	 * @brief Gets the brightness level relative to the brightness value.
	 * 
	 * @return BrightnessLevel The computed brightness level.
	 */
	BrightnessLevel getBrightnessLevel();

private:
	short _pin;
	int _rawReading;
};

#endif