#ifndef COMMANDS_H
#define COMMANDS_H

#include <avr/io.h>
#include <util/delay.h>

void BM83_Power_On(void);
void BM83_Power_Off(void);
void trigger_bluetooth_pairing(void);

#endif