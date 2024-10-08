#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <util/delay.h>

#define POWER_ON_PIN PD2                // Pin for powering on
#define POWER_OFF_PIN PD3               // Pin for powering off
#define LED_POWER_PIN PB5               // Atmega LED
//#define LED_BUTTON_PIN PD6              // Second LED for button press indication
#define BUTTON_PIN PB7                  // Button pin for triggering pairing
#define MFB PB0                        	       // Pin to control MFB for BM83
#define DEBOUNCE_DELAY_MS 50

#define F_CPU 8000000UL
#define BAUD 38400
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1)

volatile uint8_t adc_param = 0;         // Scaled volume parameter
volatile uint8_t last_adc_param = 0;    // Store last volume value
volatile uint16_t adc_value = 0;        // Store ADC result
char buffer[10];

typedef enum {
	POWER_OFF,
	POWER_ON
} PowerState;
PowerState currentState = POWER_OFF;

void USART_Init(unsigned int BAUD1) {
	UBRR0H = (unsigned char)(BAUD1 >> 8);
	UBRR0L = (unsigned char)BAUD1;
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);         // 8-bit data
}

void Power_Init(void) {
	DDRB |= (1 << LED_POWER_PIN);       // Output
	PORTB &= ~(1 << LED_POWER_PIN);     // Initialize OFF

	//DDRB |= (1 << MFB);                 // Set PB0 as output

	// Power on pin
	DDRD &= ~(1 << POWER_ON_PIN);       // Input
	PORTD |= (1 << POWER_ON_PIN);       // Enable pull-up

	// Power off pin
	DDRD &= ~(1 << POWER_OFF_PIN);      // Input
	PORTD |= (1 << POWER_OFF_PIN);      // Enable pull-up

	// External interrupts configuration
	EICRA |= (1 << ISC01) | (1 << ISC00);  // Trigger on rising edge for INT0 (power on)
	EIMSK |= (1 << INT0);               // Enable INT0 for power on

	EICRA |= (1 << ISC11) | (1 << ISC10);  // Trigger on rising edge for INT1 (power off)
	EIMSK |= (1 << INT1);
}

void ADC_Init() {
	ADMUX = (1 << REFS0);  // Set reference voltage to AVcc (5V), select ADC0
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);  // Enable ADC, prescaler 128
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

void USART_SendString(const char *str) {
	while (*str) {
		USART_Transmit(*str++);
	}
}

void BM83_Power_On(void) {
	BM83_Send_Power_Command(0x02, 0x00, 0x51); //power on commands
	_delay_ms(20);
	BM83_Send_Power_Command(0x02, 0x00, 0x52);
	_delay_ms(20);
}

void BM83_Power_Off(void) {
	BM83_Send_Power_Command(0x02, 0x00, 0x53);
	_delay_ms(20);
	BM83_Send_Power_Command(0x02, 0x00, 0x54);
	_delay_ms(20);
}

void turn_on_LED() {
	PORTB |= (1 << LED_POWER_PIN); // Turn on LED
}

void turn_off_LED() {
	PORTB &= ~(1 << LED_POWER_PIN); // Turn off LED
}

// Function to trigger Bluetooth pairing mode on the BM83 after button has been pressed
void trigger_bluetooth_pairing() {
	if (currentState == POWER_ON){
		BM83_Send_Power_Command(0x02, 0x00, 0x5D);
		_delay_ms(20);
	}
}

// Function to initialize the GPIO for the button
void initialize_gpio() {
	DDRB &= ~(1 << BUTTON_PIN);  // Set BUTTON_PIN (PB7) as input
	PORTB |= (1 << BUTTON_PIN);  // Enable pull-up resistor on BUTTON_PIN

	// Enable interrupt on falling edge (button press)
	PCICR |= (1 << PCIE0);   // Enable Pin Change Interrupts for PCINT[7:0]
	PCMSK0 |= (1 << PCINT7); // Enable PCINT7 (PB7) to trigger interrupt
}

uint16_t ADC_Read() {
	ADCSRA |= (1 << ADSC);  // Start ADC conversion
	while (ADCSRA & (1 << ADSC));  // Wait for conversion to finish
	return ADC;  // Return ADC result
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

ISR(ADC_vect) {
	// Check if the volume has changed by at least 1
	if (adc_param >= last_adc_param + 2 || adc_param <= last_adc_param - 2) {
		Volume_Command(adc_param); // Send volume change command
		last_adc_param = adc_param; // Update last known value
	}
}

// ISR for button press on PB7
ISR(PCINT0_vect) {
	// Check if PB7 caused the interrupt (button press)
	if (!(PINB & (1 << BUTTON_PIN))) {
		trigger_bluetooth_pairing();  // Trigger pairing when the button is pressed
		turn_on_LED();  // Turn on button LED to indicate button press

		// Simple delay to avoid multiple triggers during button bounce
		_delay_ms(50);
		
		// Wait for button release before allowing another interrupt
		while (!(PINB & (1 << BUTTON_PIN)));
	}
}

//might need to switch interrupt 1 and interrupt 0

ISR(INT0_vect) {
	_delay_ms(DEBOUNCE_DELAY_MS);   // Wait for signal to stabilize
	if (PIND & (1 << POWER_ON_PIN)) // Check if it's still in the expected state
	{
		currentState = POWER_ON;
		//turn_on_LED();
		BM83_Power_On();
	}
}

ISR(INT1_vect) {
	_delay_ms(DEBOUNCE_DELAY_MS);   // Wait for signal to stabilize
	if (PIND & (1 << POWER_OFF_PIN)) // Check if it's still in the expected state
	{
		currentState = POWER_OFF;
		//turn_off_LED();
		BM83_Power_Off();
	}
}

int main(void) {
	USART_Init(BAUDRATE);                   //Initalize Baudrate with USART
	Power_Init();                           //Initalize LED, pins, and interrupts
	initialize_gpio();
	ADC_Init();
	
	sei();

	while (1) {
		adc_value = ADC_Read();
		adc_param = adc_value / 8;  // Scale down from 1023 to 127
	}
}