#include "Commands.h"
#include "USART.h"
#include "Globals.h"

void BM83_Power_On(void) {
	BM83_Send_Power_Command(0x02, 0x00, 0x51); 
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

void trigger_bluetooth_pairing() {
	if (currentState == POWER_ON){
		BM83_Send_Power_Command(0x02, 0x00, 0x5D);
		_delay_ms(20);
	}
}