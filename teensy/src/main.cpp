/**
 * @file main.cpp
 * @date 2022-10-09
 * @author John Madden (jtmadden@ucsc.edu)
 *
 * Main executable for Soil Power Sensor Firmware
 *
 * The firmware handles the conversion from analog signals to digital values.
 * Voltage, current and temperature can be measured using serial commands. For
 * temperature measurements, the BME280 sensor must be connected via I2C. The
 * voltage and current values are averaged over #NUM_SAMPLES.
 *
 * The serial commands being sent to the teensy must end with "\n" character
 * making it compatable with "\n" or "\r\n" line endings. The returned serial
 * value always will end with "\r\n" as described in the Arduino
 * Serial.println() command.
 *
 * Commands:
 * 	check	Checks the connection to the firmware. Returns "ok".
 * 	info	Gets compiler info for version of firmware.
 * 	v		Measures voltage.
 * 	i		Measures current.
 * 	t		Measures temperature.
 * 	cont	Preform continuous measurements of voltage, current, temp in the form "v,c,t"
 * 	stop	Stop continuous measurements
 */

#include <stdio.h>

#include <Arduino.h>

#include <Adafruit_BME280.h>

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

typedef struct
{
	/** Average of measurements */
	double avg;
	/** Number of samples */
	unsigned int n;
} meas_t;

/** States of control fsm */
typedef enum
{
	/** Waiting for command */
	IDLE,
	/** Check communication */
	CHECK,
	/** Get compile info */
	INFO,
	/** Measure voltage */
	VOLT,
	/** Measure current */
	CURR,
	/** Measure temperature */
	TEMP,
	/** Continuous measurement */
	CONT,
} control_states;

/** Current state for control fsm */
control_states control_state;

Adafruit_BME280 bme280;

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
	return ((double)val / (1 << res)) * ref;
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
void read_voltage(int pin, meas_t *meas)
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
		meas->avg = (((double)meas->n) - 1.) / ((double)meas->n) * meas->avg;
		meas->avg += (1. / (double)meas->n) * val;
	}
}

void control_run(void)
{
	// Stay in current state unless otherwise changed
	control_states next_state = control_state;

	// Buffer to store measurements
	char meas[256];

	switch (control_state)
	{
	case CHECK:
		Serial.println("ok");
		next_state = IDLE;
		break;

	case INFO:
		Serial.println(DEBUG_STR);
		next_state = IDLE;
		break;

	case VOLT:
		digitalWrite(LED_BUILTIN, HIGH);

		meas_t v = {};

		while (v.n < NUM_SAMPLES)
		{
			read_voltage(PIN_V, &v);
		}
		sprintf(meas, "%f", v.avg);
		Serial.println(meas);

		digitalWrite(LED_BUILTIN, LOW);

		next_state = IDLE;
		break;
	
	case CURR:
		digitalWrite(LED_BUILTIN, HIGH);

		meas_t i = {};

		while (i.n < NUM_SAMPLES)
		{
			read_voltage(PIN_I, &i);
		}
		sprintf(meas, "%f", i.avg);
		Serial.println(meas);

		digitalWrite(LED_BUILTIN, LOW);

		next_state = IDLE;
		break;

	case TEMP:
		// Take measurement and return to sleep
		bme280.takeForcedMeasurement();
		float t = bme280.readTemperature();
		sprintf(meas, "%f", t);
		Serial.println(meas);

		next_state = IDLE;
		break;

	case CONT:
		meas_t v = {};
		meas_t i = {};
		float t;

		while (v.n < NUM_SAMPLES)
		{
			read_voltage(PIN_V, &v);
			read_voltage(PIN_I, &i);
		}

		bme280.takeForcedMeasurement();
		t = bme280.readTemperature();

		sprintf(meas, "%f,%f,%f", v.avg, i.avg, t);
		Serial.println(meas);

		break;

	default:
		break;
	}

	control_state = next_state;
}

void setup()
{
	// Initialize serial
	Serial.begin(9600);
	while (!Serial)
		;

	// Set ADC to 16 bits
	analogReadResolution(ADC_READ_RES);

	// Connect to bme280
	if (!bme280.begin())
	{
		Serial.println("Could not find valid BME280 sensor, check wiring");
		while (1)
			;
	}

	// Only measure temperature in forced mode to conserve power
	bme280.setSampling(
		Adafruit_BME280::MODE_FORCED,
		Adafruit_BME280::SAMPLING_X1,
		Adafruit_BME280::SAMPLING_NONE,
		Adafruit_BME280::SAMPLING_NONE,
		Adafruit_BME280::FILTER_OFF);

	// initialize LED digital pin as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	// turn the LED on (HIGH is the voltage level)
	digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
	if (Serial.available() > 0)
	{
		String cmd = Serial.readStringUntil('\n');
		cmd.trim();
		cmd.toLowerCase();

		if (cmd == "check")
		{
			control_state = CHECK;
		}
		else if (cmd == "info")
		{
			control_state = INFO;
		}
		else if (cmd == "v")
		{
			control_state = VOLT;
		}
		else if (cmd == "i")
		{
			control_state = CURR;
		}
		else if (cmd == "t")
		{
			control_state = TEMP;
		}
		else if (cmd == "cont")
		{
			control_state = CONT;
		}
		else if (cmd == "stop")
		{
			control_state = IDLE;
		}
	}

	control_run();
}
