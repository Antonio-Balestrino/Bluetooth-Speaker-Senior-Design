/*
 * GccApplication1.c
 *
 * Created: 9/20/2024 11:15:51 AM
 * Author : Ponsai
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#define F_CPU 8000000UL
#define BAUD 38400
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1)

volatile uint8_t adc_param = 0;         // Scaled volume parameter
volatile uint8_t last_adc_param = 0;    // Store last volume value
volatile uint16_t adc_value = 0;        // Store ADC result
char buffer[10];

// Initialize UART
void USART_Init(unsigned int baud) {
	UBRR0H = (unsigned char)(baud >> 8);   // Set baud rate
	UBRR0L = (unsigned char)baud;
	UCSR0B = (1 << TXEN0);  // Enable transmitter
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-bit data
}

void ADC_Init() {
	// Set reference voltage to AVcc (5V) and select ADC0
	ADMUX = (1 << REFS0);

	// Enable ADC, set prescaler to 128 for 125 kHz ADC clock, enable ADC interrupt
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);
}

uint16_t ADC_Read() {
	ADCSRA |= (1 << ADSC);  // Start ADC conversion
	while (ADCSRA & (1 << ADSC));  // Wait for conversion to finish
	return ADC;
}

void USART_Transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty transmit buffer
	UDR0 = data;  // Put data into buffer, sends the data
}

void USART_SendString(const char *str) {
	while (*str) {
		USART_Transmit(*str++);
	}
}

void Volume_Command(unsigned char parameter) {
	uint8_t packet[] = {0xAA, 0x00, 0x07, 0x23, 0x00, 0x01, 0x03, parameter, 0x00, 0x00, 0x00};

	// Calculate and set checksum
	packet[10] = ~(packet[1] + packet[2] + packet[3] + packet[4] + 
				packet[5] + packet[6] + packet[7] + packet[8]+ packet[9]) + 1;

	/*for (int i = 0; i < 10; i++) {
		snprintf(buffer, sizeof(buffer), "%u\r\n", packet[i]);  // Convert ADC value to string
		USART_SendString(buffer);  // Send value over UART
		_delay_ms(500);  // Delay for readability in PuTTY
	}*/
	
	snprintf(buffer, sizeof(buffer), "%u\r\n", parameter);  // Convert ADC value to string
	USART_SendString(buffer);  // Send value over UART
}

void BM83_SendCommand(uint8_t parameter) {
	uint8_t packet[] = {0xAA, 0x00, 0x03, 0x02, 0x00, parameter, 0x00};

	// Calculate and set checksum
	packet[6] = ~(packet[1] + packet[2] + packet[3] + packet[4] + packet[5]) + 1;

	// Send packet to BM83
	for (uint8_t i = 0; i < sizeof(packet); i++) {
		USART_Transmit(packet[i]);
	}
}

void BM83_SendCommand_Pair(uint8_t opcode, uint8_t parameter) {
	uint8_t packets[] = {0xAA, 0x00, 0x02, opcode, parameter, 0x00};

	// Calculate checksum and set it in the packet
	packets[5] = ~(packets[1] + packets[2] + packets[3] + packets[4]) + 1;
	// Add checksum later maybe

	// Send the packet to the BM83
	for (uint8_t i = 0; i < sizeof(packets); i++) {
		USART_Transmit(packets[i]);
	}
}

void BM83_Init(void) {
	// Send command to enter command mode
	BM83_SendCommand(0x51); //power on commands
	_delay_ms(100);
	BM83_SendCommand(0x52);
	_delay_ms(100);
	BM83_SendCommand_Pair(0x02, 0x5D);
}
	
/*ISR(ADC_vect) {
	// Check if the volume has changed by at least 1
	if (adc_param >= last_adc_param + 2 || adc_param <= last_adc_param - 2) {
		Volume_Command(adc_param); // Send volume change command
		last_adc_param = adc_param; // Update last known value
	}
}*/

int main(void) {
	USART_Init(BAUDRATE);  // Initialize UART
	ADC_Init();  // Initialize ADC
	
	sei();
	BM83_Init();

	while (1) {
		
		//adc_value = ADC_Read();
		//adc_param = adc_value / 8;  // Scale down from 1023 to 127

	}
}

