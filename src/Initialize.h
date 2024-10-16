// Initialize.h
#ifndef INITIALIZE_H
#define INITIALIZE_H

#include <avr/io.h>

#define POWER_ON_PIN PD2
#define POWER_OFF_PIN PD3
#define LED_POWER_PIN PB5
#define BUTTON_PIN PB7


void Power_Init(void);
void ADC_Init(void);
uint16_t ADC_Read(void);
void turn_on_LED(void);
void turn_off_LED(void);

#endif