#include "Globals.h"

volatile uint8_t adc_param = 0;      
volatile uint8_t last_adc_param = 0;   
volatile uint16_t adc_value = 0;        

PowerState currentState = POWER_OFF;     // Initialize to POWER_OFF