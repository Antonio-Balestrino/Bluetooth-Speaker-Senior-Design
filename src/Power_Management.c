#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define POWER_ON_PIN PD2  // Pin for powering on
#define POWER_OFF_PIN PD3  // Pin for powering off
#define LED_POWER_PIN PB5

typedef enum {
    POWER_OFF,
    POWER_ON
} PowerState;
PowerState currentState = POWER_OFF;

void turn_on_LED() {
    PORTB |= (1 << LED_POWER_PIN); // Turn on LED
}

void turn_off_LED() {
    PORTB &= ~(1 << LED_POWER_PIN); // Turn off LED
}

void Power_Init(void) {
    DDRB |= (1 << LED_POWER_PIN);       // Output
    PORTB &= ~(1 << LED_POWER_PIN);     // Initialize OFF

    // Power on pin
    DDRD &= ~(1 << POWER_ON_PIN);       // Input
    PORTD |= (1 << POWER_ON_PIN);       // Enable pull-up

    // Power off pin
    DDRD &= ~(1 << POWER_OFF_PIN);      // Input
    PORTD |= (1 << POWER_OFF_PIN);      // Enable pull-up

    // External interrupts configuration
    EICRA |= (1 << ISC00) | (1 << ISC01);  // Trigger on rising edge for power on
    EIMSK |= (1 << INT0);                   // Enable INT0 for power on

    EICRA |= (1 << ISC10) | (1 << ISC11);  // Trigger on rising edge for power off
    EIMSK |= (1 << INT1);                   // Enable INT1 for power off
}

ISR(INT0_vect) { 
    currentState = POWER_ON;
}

ISR(INT1_vect) { 
    currentState = POWER_OFF;
}

void sleep() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode
    sleep_enable();                       // Enable sleep mode
    sleep_cpu();                         // Go to sleep
    sleep_disable();                      // Disable sleep mode after waking up
}

int main(void) {
    Power_Init();
    sei();

    while (1) {
        if (currentState == POWER_OFF) {
			turn_off_LED();
            sleep();
        }
		else{
			turn_on_LED();
		}
    }
}
