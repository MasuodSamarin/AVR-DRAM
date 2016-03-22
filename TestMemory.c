#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "usart.h"


void main(void)
{
	uart_init(BAUD_CALC(115200));
	
	RefreshTimerInt(void);
	MemoryInit(void);

	sei();

	

	while (1)
	{

	}
}