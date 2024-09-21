#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>

// Define clock frequency and baud rate for UART
#define F_CPU 16000000UL
#define BAUD 115200
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1)

// Define pins for the button and LED
#define BUTTON_PIN PB7  // Button already on the ATmega (connected to PB7)
#define LED_PIN PB5     // Assuming the LED is connected to pin PB5

// Hypothetical opcodes for pairing command
#define PAIRING_OPCODE 0x01  
#define PAIRING_PARAM  0x00  

// Function to initialize UART communication
void USART_Init(unsigned int BAUD1) {
    UBRR0H = (unsigned char)(BAUD1 >> 8);  // Set baud rate high byte
    UBRR0L = (unsigned char)BAUD1;         // Set baud rate low byte
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);  // Enable TX and RX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data
}

// Function to transmit a single byte over UART
void USART_Transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty transmit buffer
    UDR0 = data;  // Send data
}

// Function to receive a single byte over UART
unsigned char USART_Receive(void) {
    while (!(UCSR0A & (1 << RXC0)));  // Wait for data to be received
    return UDR0;  // Return received data
}

// Function to send a command to the BM83 module via UART
void BM83_SendCommand(uint8_t opcode, uint8_t parameter) {
    uint8_t packet[] = {0xAA, 0x00, 0x02, opcode, parameter, 0x00};

    // Calculate checksum and set it in the packet
    packet[5] = ~(packet[1] + packet[2] + packet[3] + packet[4]) + 1;

    // Send the packet to the BM83
    for (uint8_t i = 0; i < sizeof(packet); i++) {
        USART_Transmit(packet[i]);
    }
}

// Function to initialize the BM83 module (including power-on commands)
void BM83_Init(void) {
    // Send command to enter command mode
    BM83_SendCommand(0x00, 0x51); //power on commands
    _delay_ms(100);
    BM83_SendCommand(0x00, 0x52);
    _delay_ms(100);  // Delay for command mode entry
    BM83_SendCommand(0x08, 0x01);
    _delay_ms(100);

}

// Function to initialize the GPIO for the button
void initialize_gpio() {
    DDRB &= ~(1 << BUTTON_PIN);  // Set BUTTON_PIN (PB7) as input
    PORTB |= (1 << BUTTON_PIN);  // Enable pull-up resistor on BUTTON_PIN
}

// Function to initialize the LED pin as output
void init_LED() {
    DDRB |= (1 << LED_PIN);  // Set LED_PIN (PB5) as output
}

// Function to turn on the LED
void turn_on_LED() {
    PORTB |= (1 << LED_PIN);  // Set LED_PIN high
}

// Function to turn off the LED
void turn_off_LED() {
    PORTB &= ~(1 << LED_PIN);  // Set LED_PIN low
}

// Function to check if the button is pressed (with debouncing)
bool is_button_pressed() {
    if (!(PINB & (1 << BUTTON_PIN))) {  // Check if button is pressed (active low)
        _delay_ms(50);  // Simple debounce delay
        if (!(PINB & (1 << BUTTON_PIN))) {  // Check again after debounce delay
            return true;
        }
    }
    return false;
}

// Function to trigger Bluetooth pairing mode on the BM83 after button has been pressed
void trigger_bluetooth_pairing() {
    BM83_SendCommand(0x02, 0x5D);  // MMI_Action command to fast enter pairing mode
}

int main(void) {
    // Initialize UART, LED, BM83 module, and button GPIO
    USART_Init(BAUDRATE);
    init_LED();
    BM83_Init();
    initialize_gpio();

    while (1) {
        if (is_button_pressed()) {
            trigger_bluetooth_pairing();  // Trigger pairing when the button is pressed
            turn_on_LED();  // Turn on LED to indicate pairing mode is triggered

            // Wait for button release to avoid repeated triggers
            while (is_button_pressed());

            turn_off_LED();  // Turn off LED after pairing mode
        }
    }
}
