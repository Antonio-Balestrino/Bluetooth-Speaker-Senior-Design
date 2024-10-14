#ifndef GLOBALS_H
#define GLOBALS_H

#include <avr/io.h>
#include <stdint.h>

extern volatile uint8_t adc_param;         // Scaled volume parameter
extern volatile uint8_t last_adc_param;    // Store last volume value
extern volatile uint16_t adc_value;        // Store ADC result

typedef enum {
	POWER_OFF,
	POWER_ON
} PowerState;

extern PowerState currentState;

#endif