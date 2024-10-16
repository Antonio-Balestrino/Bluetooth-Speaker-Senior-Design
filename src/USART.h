#ifndef USART_H
#define USART_H

#include <avr/io.h>

#define F_CPU 8000000UL                             // Clock speed
#define BAUD 38400                                  // Baud rate
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1)      // Baud rate register value

void USART_Init(unsigned int BAUD1);
void USART_Transmit(unsigned char data);
unsigned char USART_Receive(void);
void BM83_Send_Power_Command(uint8_t opcode, uint8_t parameter1, uint8_t parameter2);
void Volume_Command(unsigned char parameter);

#endif