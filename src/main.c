#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>

#include "Globals.h"
#include "USART.h"
#include "Initialize.h"
#include "Commands.h"
#include "Interrupts.h"

int main(void) {
	USART_Init(BAUDRATE);                   //Initalize Baudrate with USART
	Power_Init();                           //Initalize LED, pins, and interrupts
	ADC_Init();
	sei();

	while (1) {
		adc_value = ADC_Read();
		adc_param = adc_value / 8;  		// Scale down from 1023 to 127
	}
}