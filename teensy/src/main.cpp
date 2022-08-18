/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <stdio.h>

#include "Arduino.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

/** ADC input resolution */
#define ADC_READ_RES 16

/** ADC Effective number of bits due to noise */
#define ADC_EFF_BITS 13

#define ADC_EFF_MASK (0xFFFF << (ADC_READ_RES - ADC_EFF_BITS))

/** Reference voltage for ADC */
#define VREF 3.3

/** ADC Pin used for measuring current */
#define PIN_I A14
/** ADC Pin used for measuring voltage */
#define PIN_V A15

/** Number of samples to average */
#define NUM_SAMPLES 10000

/**
 * @brief Converts ADC reading to voltage based of reference voltage and ADC
 * resolution
 *
 * @param val ADC Value
 * @param ref Reference voltage
 * @param res ADC Resolution
 * @return Equivalent voltage
 */
inline double conv_ADC(int val, double ref, int res)
{
	return ((double) val / (1 << res)) * ref;
}

void setup()
{
	// Initialize serial
	Serial.begin(9600);
	while (!Serial);

	// Bring debug string
	char debug_str[256];
	sprintf(debug_str, "Soil Power Sensor Calibration, compiled on %s, %s", __DATE__, __TIME__);
	Serial.println(debug_str);

	// Set ADC to 16 bits
	analogReadResolution(ADC_READ_RES);

	// initialize LED digital pin as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	// turn the LED on (HIGH is the voltage level)
	digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
	char meas[256];

	// Number of current samples
	unsigned int n = 0;

	// Mean current
	double mean_i;
	// Mean voltage
	double mean_v;

	while (n < NUM_SAMPLES)
	{
		// Read pins
		int raw_i = analogRead(PIN_I);
		int raw_v = analogRead(PIN_V);

		// Mask to effective bits to remove ADC noise
		int eff_i = raw_i & ADC_EFF_MASK;
		int eff_v = raw_v & ADC_EFF_MASK;

		// Convert to readings
		double i = conv_ADC(eff_i, VREF, ADC_READ_RES);
		double v = conv_ADC(eff_v, VREF, ADC_READ_RES);

		// If on first iteration store directly in mean
		if (n == 0)
		{
			mean_i = i;
			mean_v = v;
			++n;
		}
		else
		{
			++n;
			mean_i = (((double) n) - 1.) / ((double) n) * mean_i + (1. / (double) n) * i;
			mean_v = (((double) n) - 1.) / ((double) n) * mean_v + (1. / (double) n) * v;
		}
	}


	// Print
	sprintf(meas, "Eff V: %f, I: %f", mean_v, mean_i);
	Serial.println(meas);
}
