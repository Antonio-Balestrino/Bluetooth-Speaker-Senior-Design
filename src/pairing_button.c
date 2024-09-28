#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// Define clock frequency and baud rate for UART
#define F_CPU 8000000UL // 8 MHz clock
#define BAUD 38400
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1)

// Define pins for the button and LED
#define BUTTON_PIN PB7  // Button already on the ATmega (connected to PB7)
#define LED_PIN PB5     // Assuming the LED is connected to pin PB5

// Global variables for resend logic
volatile uint8_t command_sent = 0; // Flag indicating if a command was sent
volatile uint8_t ack_received = 0; // Flag indicating if an acknowledgment was received

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
	// Add checksum later maybe

	// Send the packet to the BM83
	for (uint8_t i = 0; i < sizeof(packet); i++) {
		USART_Transmit(packet[i]);
	}
}

// Function to get an event from the BM83 module
uint8_t* BM83_Event(void) {
	// Define a buffer for the event header and data
	uint8_t header[3];
	
	// Read the header
	header[0] = USART_Receive();  // Expected to be 0xAA
	header[1] = USART_Receive();  // High byte of length (LH)
	header[2] = USART_Receive();  // Low byte of length (LL)

	// Calculate the number of bytes in the event
	uint16_t length = (header[1] << 8) | (header[2] & 0xFF);

	// Allocate memory dynamically based on the length of the data plus the header size
	uint8_t *event = (uint8_t*)malloc((3 + length) * sizeof(uint8_t)); // Correct casting here
	
	// Copy the header to the event buffer
	event[0] = header[0];
	event[1] = header[1];
	event[2] = header[2];

	// Read the event data
	for (uint16_t i = 0; i < length; i++) {
		event[3 + i] = USART_Receive();
	}

	return event; // Return the event data
}

// Function to send acknowledgment for the event ID
void BM83_AcknowledgeEvent(uint8_t eventID) {
	uint8_t ackPacket[] = {0xAA, 0x00, 0x02, 0x14, eventID, 0x00};
	
	// Calculate checksum for acknowledgment
	ackPacket[5] = ~(ackPacket[1] + ackPacket[2] + ackPacket[3] + ackPacket[4]) + 1;

	// Transmit the acknowledgment packet
	for (uint8_t i = 0; i < sizeof(ackPacket); i++) {
		USART_Transmit(ackPacket[i]);
	}
}

// Function to determine the event type and respond if necessary
void BM83_GetEventType(uint8_t *event) {
	// Calculate the length of the event (header + data length)
	uint16_t length = (event[1] << 8) | (event[2] & 0xFF);

	// Ensure that the length is at least 5 bytes (header + event ID + minimum payload)
	if (length < 5) {
		return;  // Invalid event length, no further action needed
	}

	// Extract the event ID from the event data
	uint8_t eventID = event[3];

	// Check if the event is valid (not an error indication)
	if (eventID != 0) {
		// Send acknowledgment for valid event IDs
		BM83_AcknowledgeEvent(eventID);
	}
}

void BM83_Init(void) {
	// Send command to enter command mode
	BM83_SendCommand(0x02, 0x51); // power on command
	_delay_ms(100);
	
	// Wait for acknowledgment before proceeding
	uint8_t *event = BM83_Event(); // Wait for the event after powering on
	BM83_GetEventType(event); // Check for acknowledgment or event type

	BM83_SendCommand(0x08, 0x01); // Other command (as needed)
	_delay_ms(100);
	
	// Wait for acknowledgment before proceeding
	event = BM83_Event(); // Wait for the event after the command
	BM83_GetEventType(event); // Check for acknowledgment or event type
}

// Function to initialize the GPIO for the button
void initialize_gpio() {
	DDRB &= ~(1 << BUTTON_PIN);  // Set BUTTON_PIN (PB7) as input
	PORTB |= (1 << BUTTON_PIN);  // Enable pull-up resistor on BUTTON_PIN

	// Enable interrupt on falling edge (button press)
	PCICR |= (1 << PCIE0);   // Enable Pin Change Interrupts for PCINT[7:0]
	PCMSK0 |= (1 << PCINT7); // Enable PCINT7 (PB7) to trigger interrupt
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

// Function to initialize Timer1 for 200ms timeout
void init_timer() {
	TCCR1B |= (1 << WGM12);             // Configure timer 1 for CTC mode
	TIMSK1 |= (1 << OCIE1A);            // Enable CTC interrupt
	OCR1A = 15624;                      // Set CTC compare value for 200ms with 8MHz clock and 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10); // Set 1024 prescaler
}

// Function to trigger Bluetooth pairing mode on the BM83 after button has been pressed
void trigger_bluetooth_pairing() {
	BM83_SendCommand(0x02, 0x5D);  // MMI_Action command to fast enter pairing mode

	// Wait for acknowledgment after triggering pairing mode
	uint8_t *event = BM83_Event(); // Wait for the event after sending the command
	BM83_GetEventType(event); // Check for acknowledgment or event type

	ack_received = 0; // Clear acknowledgment flag
	command_sent = 1; // Set command sent flag
	init_timer(); // Start the 200ms timer
}

// ISR for button press on PB7
ISR(PCINT0_vect) {
	// Check if PB7 caused the interrupt (button press)
	if (!(PINB & (1 << BUTTON_PIN))) {
		trigger_bluetooth_pairing();  // Trigger pairing when the button is pressed
		turn_on_LED();  // Turn on LED to indicate pairing mode is triggered

		// Simple delay to avoid multiple triggers during button bounce
		_delay_ms(50);
		
		// Wait for button release before allowing another interrupt
		while (!(PINB & (1 << BUTTON_PIN)));
		
		turn_off_LED();  // Turn off LED after pairing mode
	}
}

// ISR for UART RX complete
ISR(USART_RX_vect) {
	uint8_t receivedByte = USART_Receive();
	if (receivedByte == 0x00) { // Assuming 0x00 is ACK (you can change this as needed)
		ack_received = 1; // Set acknowledgment received flag
		command_sent = 0; // Clear command sent flag
		TCCR1B &= ~((1 << CS12) | (1 << CS10)); // Stop the timer
	}
}

// ISR for Timer1 compare match, triggers when 200ms timeout occurs
ISR(TIMER1_COMPA_vect) {
	if (command_sent && !ack_received) { // If command was sent but not acknowledged
		trigger_bluetooth_pairing(); // Resend the command
	}
}


// ISR for receiving data from the BM83 module to be added

int main(void) {
	// Initialize UART, LED, BM83 module, and button GPIO
	USART_Init(BAUDRATE);
	init_LED();
	BM83_Init();
	initialize_gpio();

	sei(); // Enable global interrupts

	while (1) {
		// Main loop does nothing; everything is handled by the ISR
	}
}