/*
 * GccApplication1.c
 *
 * Created: 9/20/2024 11:15:51 AM
 * Author : Ponsai
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 16000000UL
#define BAUD 115200
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1)

volatile uint8_t adc_param = 0;         // Scaled volume parameter
volatile uint8_t last_adc_param = 0;    // Store last volume value
volatile uint16_t adc_value = 0;        // Store ADC result

void USART_Init(unsigned int BAUD1) {
	UBRR0H = (unsigned char)(BAUD1 >> 8);
	UBRR0L = (unsigned char)BAUD1;
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // Enable RX Complete Interrupt
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data
}

void USART_Transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
	UDR0 = data; // Put data into buffer, sends the data
}

void ADC_Init() {
	// Set reference voltage to AVcc (5V) and select ADC0
	ADMUX = (1 << REFS0);

	// Enable ADC, set prescaler to 128 for 125 kHz ADC clock, enable ADC interrupt
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);
}

void Volume_Command(uint8_t parameter) {
	uint8_t packet[] = {0xAA, 0x00, 0x07, 0x23, 0x00, 0x01, 0x03, parameter, 0x00, 0x00};

	// Calculate and set checksum
	packet[9] = ~(packet[0] + packet[1] + packet[2] + packet[3] +
	packet[4] + packet[5] + packet[6] + packet[7] + packet[8]) + 1;
	
	// Transmit the packet over UART
	for (int i = 0; i < 10; i++) {
		USART_Transmit(packet[i]);
	}
}

ISR(ADC_vect) {
	adc_value = ADC;
	adc_param = adc_value / 8;  // Scale down from 1023 to 127

	// Check if the volume has changed by at least 1
	if (abs(adc_param - last_adc_param) >= 1) {
		Volume_Command(adc_param); // Send volume change command
		last_adc_param = adc_param; // Update last known value
	}

	ADCSRA |= (1 << ADSC);  // Start the next ADC conversion
}

int main(void)
{
	USART_Init(BAUDRATE);  
	ADC_Init();            

	sei();                 // Enable global interrupts

	// Start the first ADC conversion
	ADCSRA |= (1 << ADSC);

	while (1) {
	}
}

