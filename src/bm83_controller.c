#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

#define F_CPU 16000000UL
#define BAUD 115200
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1)

#define LED_PIN PB5 // Assuming the LED is connected to pin PB5

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

unsigned char USART_Receive(void) {
    while (!(UCSR0A & (1 << RXC0))); // Wait for data to be received
    return UDR0; // Get and return received data from buffer
}

void BM83_SendCommand(uint8_t opcode, uint8_t parameter) {
    uint8_t packet[] = {0xAA, 0x00, 0x02, opcode, parameter, 0x00};

    // Calculate and set checksum
    packet[5] = ~(packet[1] + packet[2] + packet[3] + packet[4]) + 1;

    // Send packet to BM83
    for (uint8_t i = 0; i < sizeof(packet); i++) {
        USART_Transmit(packet[i]);
    }
}

void BM83_Init(void) {
    // Send command to enter command mode
    BM83_SendCommand(0x00, 0x51); //power on commands
    _delay_ms(100);
    BM83_SendCommand(0x00, 0x52);
    _delay_ms(100);  // Delay for command mode entry
    BM83_SendCommand(0x08, 0x01);
    _delay_ms(100);

    // Other initialization steps...
}

void BM83_ProcessResponse(void) {
    // Process response from BM83 module
    while (1) {
        unsigned char response = USART_Receive();
        if (response == '\n') {
            // Newline character indicates end of response
            break;
        }
        // Print received character to output window
        printf("%c", response);
    }
}

void init_LED() {
    DDRB |= (1 << LED_PIN); // Set LED pin as output
}

void turn_on_LED() {
    PORTB |= (1 << LED_PIN); // Turn on LED
}

void turn_off_LED() {
    PORTB &= ~(1 << LED_PIN); // Turn off LED
}

int main(void) {
    USART_Init(BAUDRATE);
    init_LED(); // Initialize LED pin

    // Initialize BM83 module
    BM83_Init();

    // Initialize UART for printf
    stdout = &uart_output;
    stderr = &uart_output;

    while (1) {
        // Other application logic here
    }
}
