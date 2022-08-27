/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <stdio.h>

#include "Arduino.h"

/** Debug string */
#define DEBUG_STR "Soil Power Sensor Calibration, compiled on " __DATE__ ", " __TIME__

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
#define NUM_SAMPLES 100


typedef struct {
	/** Average of measurements */
	double avg;
	/** Number of samples */
	unsigned int n;
} meas_t;


/**
 * @brief Converts ADC reading to voltage based of reference voltage and ADC
 * resolution.
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


/**
 * @brief Reads voltage from ADC channel
 *
 * The raw value is converted based on the reference voltage and the resolution
 * of the ADC. A mask is applied to the read ADC value to reduce the number of
 * specified by the settings. If n is 0 then the measurement is not averaged.
 *
 * @see ADC_READ_RES
 * @see ADC_EFF_MASK
 * @see VREF
 *
 * @param[in] pin Analog pin
 * @param[in/out] meas Pointer to measurement
 */
void read_voltage(int pin, meas_t * meas)
{
		// Read pins
		int raw = analogRead(pin);

		// Mask to effective bits to remove ADC noise
		int eff = raw & ADC_EFF_MASK;

		// Convert to readings
		double val = conv_ADC(eff, VREF, ADC_READ_RES);

		// If on first iteration store directly in mean
		if (meas->n == 0)
		{
			meas->avg = val;
			++meas->n;
		}
		else
		{
			++meas->n;
			meas->avg = (((double) meas->n) - 1.) / ((double) meas->n) * meas->avg;
			meas->avg += (1. / (double) meas->n) * val;
		}
}


void setup()
{
	// Initialize serial
	Serial.begin(9600);
	while (!Serial);

	// 
	// Set ADC to 16 bits
	analogReadResolution(ADC_READ_RES);

	// initialize LED digital pin as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	// turn the LED on (HIGH is the voltage level)
	digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
	// Buffer to store measurements
	char meas[256];

	String cmd = Serial.readStringUntil('\n');
	cmd.trim();
	cmd.toLowerCase();

	if (cmd == "check")
	{
		Serial.println("ok");
	}
	else if (cmd == "info")
	{
		Serial.println(DEBUG_STR);
	}
	else if (cmd == "v")
	{
		digitalWrite(LED_BUILTIN, HIGH);

		meas_t i = {};

		while (i.n < NUM_SAMPLES)
		{
			read_voltage(PIN_I, &i);
		}
		sprintf(meas, "%f", i.avg);
		Serial.println(meas);

		digitalWrite(LED_BUILTIN, LOW);
	}
	else if (cmd == "i")
	{
		digitalWrite(LED_BUILTIN, HIGH);

		meas_t v = {};

		while (v.n < NUM_SAMPLES)
		{
			read_voltage(PIN_V, &v);
		}
		sprintf(meas, "%f", v.avg);
		Serial.println(meas);
		
		digitalWrite(LED_BUILTIN, LOW);
	}
	else if (cmd == "cont")
	{
		for (;;) 
		{
			meas_t v = {};
			read_voltage(PIN_V, &v);
			sprintf(meas, "%f", v.avg);
			Serial.println(meas);
		}
	}
}
