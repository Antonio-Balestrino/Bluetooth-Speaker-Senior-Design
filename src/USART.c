#include "USART.h"

void USART_Init(unsigned int BAUD1) {
	UBRR0H = (unsigned char)(BAUD1 >> 8);           // Baud rate high byte
	UBRR0L = (unsigned char)BAUD1;                  // Baud rate low byte
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);           // Enable Tx and Rx
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);         // 8-bit data
}

void USART_Transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));               // Wait for empty transmit buffer
	UDR0 = data;                                    // Put data into buffer, sends the data
}

unsigned char USART_Receive(void) {
	while (!(UCSR0A & (1 << RXC0)));                // Wait for data to be received
	return UDR0;                                    // Get and return received data from buffer
}

void BM83_Send_Power_Command(uint8_t opcode, uint8_t parameter1, uint8_t parameter2) {
	uint8_t packet[] = {0xAA, 0x00, 0x03, opcode, parameter1, parameter2, 0x00};

	// Calculate and set checksum
	packet[6] = ~(packet[1] + packet[2] + packet[3] + packet[4] + packet[5]) + 1;

	// Send packet to BM83
	for (uint8_t i = 0; i < sizeof(packet); i++) {
		USART_Transmit(packet[i]);
	}
}

void Volume_Command(unsigned char parameter) {
	uint8_t packet[] = {0xAA, 0x00, 0x07, 0x23, 0x00, 0x01, 0x04, parameter, 0x00, 0x00, 0x00};

	// Calculate and set checksum
	packet[10] = ~(packet[1] + packet[2] + packet[3] + packet[4] +
	packet[5] + packet[6] + packet[7] + packet[8]+ packet[9]) + 1;

	for (int i = 0; i < sizeof(packet); i++) {
		USART_Transmit(packet[i]);
	}
}