#include "Initialize.h"

void Power_Init(void) {
	DDRB |= (1 << LED_POWER_PIN);           // Output
	PORTB &= ~(1 << LED_POWER_PIN);         // Initialize OFF

	// Power on pin
	DDRD &= ~(1 << POWER_ON_PIN);           // Input
	PORTD |= (1 << POWER_ON_PIN);           // Enable pull-up

	// Power off pin
	DDRD &= ~(1 << POWER_OFF_PIN);          // Input
	PORTD |= (1 << POWER_OFF_PIN);          // Enable pull-up

	// External interrupts configuration
	EICRA |= (1 << ISC01) | (1 << ISC00);   // Trigger on rising edge for INT0 (power on)
	EIMSK |= (1 << INT0);                   // Enable INT0

	EICRA |= (1 << ISC11) | (1 << ISC10);   // Trigger on rising edge for INT1 (power off)
	EIMSK |= (1 << INT1);                   // Enable INT1

    // Initialize button
	DDRB &= ~(1 << BUTTON_PIN);             // Set BUTTON_PIN (PB7) as input
	PORTB |= (1 << BUTTON_PIN);             // Enable pull-up resistor on BUTTON_PIN

	// Enable interrupt on falling edge (button press)
	PCICR |= (1 << PCIE0);                  // Enable Pin Change Interrupts for PCINT[7:0]
	PCMSK0 |= (1 << PCINT7);                // Enable PCINT7 (PB7) to trigger interrupt
}

void ADC_Init() {
	ADMUX = (1 << REFS0);                   // Set reference voltage to AVcc (5V), select ADC0
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);  // Enable ADC, prescaler 128
}

uint16_t ADC_Read() {
	ADCSRA |= (1 << ADSC);                  // Start ADC conversion
	while (ADCSRA & (1 << ADSC));           // Wait for conversion to finish
	return ADC;                             // Return ADC result
}

void turn_on_LED() {
	PORTB |= (1 << LED_POWER_PIN);      
}

void turn_off_LED() {
	PORTB &= ~(1 << LED_POWER_PIN); 
}