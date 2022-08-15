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

void setup()
{

	// Initialize serial
	Serial.begin(9600);
	while (!Serial);

	// Bring debug string
	char debug_str[256];
	sprintf(debug_str, "Soil Power Sensor Calibration, compiled on %s, %s", __DATE__, __TIME__);
	Serial.println(debug_str);


	// initialize LED digital pin as an output.
	pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
	// turn the LED on (HIGH is the voltage level)
	digitalWrite(LED_BUILTIN, HIGH);

	// wait for a second
	delay(1000);

	// turn the LED off by making the voltage LOW
	digitalWrite(LED_BUILTIN, LOW);

	// wait for a second
	delay(1000);
}
