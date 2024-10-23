#include "Interrupts.h"
#include "Commands.h"
#include "Initialize.h"
#include "Globals.h"


ISR(ADC_vect) {
	// Check if the volume has changed by at least 1
	if (adc_param >= last_adc_param + 2 || adc_param <= last_adc_param - 2) {
		Volume_Command(adc_param);              // Send volume change command
		last_adc_param = adc_param;             // Update last known value
	}
}

ISR(PCINT0_vect) {
	// Check if PB7 caused the interrupt (button press)
	if (!(PINB & (1 << BUTTON_PIN))) {
		trigger_bluetooth_pairing();            // Trigger pairing when the button is pressed
		_delay_ms(50);
		
		while (!(PINB & (1 << BUTTON_PIN)));    // Wait for button release before allowing another interrupt
	}
}

ISR(INT0_vect) { 
    _delay_ms(DEBOUNCE_DELAY_MS);               // Wait for signal to stabilize
    if (PIND & (1 << POWER_ON_PIN))             // Check if it's still in the expected state
    {
        currentState = POWER_ON;
        turn_off_LED();
        BM83_Power_On();
    }
}

ISR(INT1_vect) { 
    _delay_ms(DEBOUNCE_DELAY_MS);               
    if (PIND & (1 << POWER_OFF_PIN))            
    {
        currentState = POWER_OFF;
        turn_on_LED();
        BM83_Power_Off();
    }
}