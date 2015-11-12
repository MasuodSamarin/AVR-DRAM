#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "usart.hpp"

/*
Memory: uPD41464-12
ATmega 88
F_CPU 18432000

PC0..5: ADDR
PB4: ADDR 6
PB5: ADDR 7

PD2: OE
PD3: WE
PD4: RAS
PD5: CAS

PD6: LED1 - refresh cpu time
PD7: LED2 -

PB0..3: Data IO
*/

// refresh interrupt
#define LED1_OFF PORTD &= ~(1<<PD6);
#define	LED1_ON PORTD |= (1<<PD6);
#define	LED1_TOG PORTD ^= (1<<PD6);

#define LED2_OFF PORTD &= ~(1<<PD7);
#define	LED2_ON PORTD |= (1<<PD7);
#define	LED2_TOG PORTD ^= (1<<PD7);

#define RAS_HI PORTD |= (1<<PD4)
#define RAS_LO PORTD &= ~(1<<PD4)

#define CAS_HI PORTD |= (1<<PD5)
#define CAS_LO PORTD &= ~(1<<PD5)

#define WE_HI PORTD |= (1<<PD3)
#define WE_LO PORTD &= ~(1<<PD3)

#define OE_HI PORTD |= (1<<PD2)
#define OE_LO PORTD &= ~(1<<PD2)

USART uart0;

void TimerInt(void);
void MemoryInit(void); 	// Initialization sequence depends on datasheet of target memory

uint8_t DramRead4b(uint16_t addr);
void DramWrite4b(uint16_t addr, uint8_t dat);

uint8_t DramRead8b(uint16_t addr);
void DramWrite8b(uint16_t addr, uint8_t dat);

void DramPageRead8b(uint16_t addr, uint8_t count, uint8_t *Dst); // 4 bit memory, pagesize/2 addr mul 2
void DramPageWrite8b(uint16_t addr, uint8_t count, uint8_t *Dst); // function doesnt swap pages if reaching end of page

void DramReadModifyWrite4b(uint16_t addr, uint8_t(*ModifyFunc)(uint8_t));

uint8_t DoCallback(uint8_t tmp) { return tmp += 1; }

int main(void)
{
	DDRB |= (1 << PB4) | (1 << PB5); // h addr
	DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
	DDRC |= 0xff;

	uart0.init(BAUD_CALC(115200));
	TimerInt();
	MemoryInit();

	sei();

	_delay_ms(1000);

	uart0 << "writing\r\n";

	for (uint16_t i = 0; i < 32768; i++)
	{
		DramWrite8b(i, i);
	}

	uart0 << "read:\r\n";

	for (uint16_t i = 0; i < 128; i++)
	{
		uart0 << DramRead8b(i) << " ";
	}
	uart0 << "\r\n";

	uart0 << "pageread\r\n";

	uint8_t T[128];

	DramPageRead8b(0, 128, T);

	for (uint16_t i = 0; i < 128; i++)
	{
		uart0 << T[i] << " ";
	}

	uart0 << "\r\n\n";

	while (1)
	{
		for (uint16_t i = 0; i < 32; i++)
		{
			uart0 << DramRead8b(i) << " ";

			_delay_ms(10);
		}

		for (uint16_t i = 0; i < 32; i++)
		{
			DramReadModifyWrite4b(i, DoCallback);
		}

		uart0 << "\r\n\n";
		_delay_ms(4000);
	}

}

void TimerInt(void)
{
	TCCR0B |= (1 << CS02); // 256 // 3.5 ms refresh peroid at 18.432 MHz
	TIMSK0 |= (1 << TOIE0); // overflow
}

void MemoryInit(void)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		RAS_LO;
		_delay_us(0.12);
		RAS_HI;
		_delay_us(0.12);
	}

	_delay_us(100);
}

uint8_t DramRead4b(uint16_t addr)
{
	uint8_t tmp;

	DDRB &= ~(0b00001111); // 4 bit input
	PORTB &= ~((1 << PB4) | (1 << PB5)); // clear adress pins to write new value 

	WE_HI;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// The row address must be applied to the address input pins on the memory device for the prescribed
		// amount of time before RAS goes low and held after RAS goes low.

		PORTC = (uint8_t)(addr >> 8) & 0b00111111; // only 5 pins on PORTC
		PORTB |= (uint8_t)(addr >> 10) & 0b00110000; // rest of address pins

		// RAS must go from high to low and remain low.

		RAS_LO;

		// A column address must be applied to the address input pins on the memory device for the prescribed amount of time and held after CAS goes low.

		PORTC = (uint8_t)addr & 0b00111111; // only 5 pins on PORTC

		PORTB &= ~((1 << PB4) | (1 << PB5)); // clear bits again

		PORTB |= (uint8_t)(addr >> 2) & 0b00110000; // rest of address pins

		// WE must be set high for a read operation to occur prior to the transition of CAS, and remain high after the transition of CAS.

		// CAS must switch from high to low and remain low.

		CAS_LO;

		// OE goes low within the prescribed window of time.

		OE_LO;

		// Data appears at the data output pins of the memory device. The time at which the data appears depends on when RAS , CAS and OE went low, and when the address is supplied.

		asm volatile("nop"::); // possible bug point

		tmp = PINB & 0b00001111; // 4 bit memory

		// Before the read cycle can be considered complete, CAS and RAS must return to their inactive states.

		CAS_HI;
		RAS_HI;
	}

	return tmp;
}

void DramWrite4b(uint16_t addr, uint8_t dat)
{
	DDRB |= 0b00001111; // 4 bit output
	PORTB &= ~(0b00111111); // clear outputs and adress pins
	//PORTB &= ~((1<<PB4)|(1<<PB5)); 

	OE_HI;
	WE_LO;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// The row address must be applied to the address input pins on the memory device for the prescribed amount of time before RAS goes low and be held for a period of time.

		PORTC = (uint8_t)(addr >> 8) & 0b00111111; // only 5 pins on PORTC
		PORTB |= (uint8_t)(addr >> 10) & 0b00110000; // rest of address pins

		// RAS must go from high to low.

		RAS_LO;

		// A column address must be applied to the address input pins on the memory device for the prescribed amount of time after RAS goes low and before CAS goes low and held for the prescribed time.

		PORTC = (uint8_t)addr & 0b00111111; // only 5 pins on PORTC

		PORTB &= ~((1 << PB4) | (1 << PB5)); // clear bits again

		PORTB |= (uint8_t)(addr >> 2) & 0b00110000; // rest of address pins

		// WE must be set low for a certain time for a write operation to occur. The timing of the transitions are determined by CAS going low.

		// Data must be applied to the data input pins the prescribed amount of time before CAS goes low  and held.

		PORTB |= dat & 0b00001111;

		// CAS must switch from high to low.

		CAS_LO;

		// Before the write cycle can be considered complete, CAS and RAS must return to their inactive states.

		CAS_HI;
		RAS_HI;
	}
}

uint8_t DramRead8b(uint16_t addr)
{
	uint8_t tmp;

	tmp = (DramRead4b(addr * 2) << 4);
	tmp |= DramRead4b(addr * 2 + 1);

	return tmp;
}
void DramWrite8b(uint16_t addr, uint8_t dat)
{
	DramWrite4b(addr * 2, (dat >> 4));
	DramWrite4b(addr * 2 + 1, dat);
}

void DramPageRead8b(uint16_t addr, uint8_t count, uint8_t *Dst)
{
	addr *= 2; // for 8b mode
	DDRB &= ~(0b00001111); // 4 bit input

	WE_HI; // keep high during transmission

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		for (uint16_t i = 0; i < count;i++)
		{
			// The row address must be applied to the address input pins on the memory device for the prescribed
			// amount of time before RAS goes low and held after RAS goes low.

			PORTC = (uint8_t)(addr >> 8) & 0b00111111; // only 5 pins on PORTC

			PORTB &= ~((1 << PB4) | (1 << PB5)); // clear adress pins to write new value

			PORTB |= (uint8_t)(addr >> 10) & 0b00110000; // rest of address pins

			// RAS must go from high to low and remain low.

			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time and held after CAS goes low.

			PORTC = (uint8_t)addr & 0b00111111; // only 5 pins on PORTC

			PORTB &= ~((1 << PB4) | (1 << PB5)); // clear adress pins to write new value

			PORTB |= (uint8_t)(addr >> 2) & 0b00110000; // rest of address pins

			// WE must be set high for a read operation to occur prior to the transition of CAS, and remain high after the transition of CAS.
			// CAS must switch from high to low and remain low.

			CAS_LO;

			// OE goes low within the prescribed window of time.

			OE_LO; // can be moved outside the loop with faster memories

			// Data appears at the data output pins of the memory device. The time at which the data appears depends on when RAS , CAS and OE went low, and when the address is supplied.

			//asm volatile("nop"::); // possible bug point
			addr++;

			Dst[i] = ((PINB & 0b00001111) << 4); // 4 bit memory

			// Only CAS goes HI for fpm mode

			CAS_HI;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time and held after CAS goes low.

			PORTC = (uint8_t)addr & 0b00111111; // only 5 pins on PORTC

			PORTB &= ~((1 << PB4) | (1 << PB5)); // clear adress pins to write new value

			PORTB |= (uint8_t)(addr >> 2) & 0b00110000; // rest of address pins

			// CAS must switch from high to low and remain low.

			CAS_LO;

			// OE goes low within the prescribed window of time.

			//OE_LO; // OE is still low

			// Data appears at the data output pins of the memory device. The time at which the data appears depends on when RAS , CAS and OE went low, and when the address is supplied.

			//asm volatile("nop"::); // possible bug point
			//asm volatile("nop"::);
			addr++;

			Dst[i] |= (PINB & 0b00001111); // 4 bit memory

			// Only CAS goes HI for fpm mode

			CAS_HI;
		}

		RAS_HI; // end of fpm mode
	}

}

void DramPageWrite8b(uint16_t addr, uint8_t count, uint8_t *Dst)
{
	addr *= 2; // for 8b mode
	DDRB |= 0b00001111; // 4 bit output

	WE_LO;
	OE_HI;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		for (uint16_t i = 0; i < count;i++)
		{
			// clear outputs and adress pins

			PORTB &= ~(0b00111111);

			// The row address must be applied to the address input pins on the memory device for the prescribed amount of time before RAS goes low and be held for a period of time.

			PORTC = (uint8_t)(addr >> 8) & 0b00111111; // only 5 pins on PORTC
			PORTB |= (uint8_t)(addr >> 10) & 0b00110000; // rest of address pins

			// RAS must go from high to low.

			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time after RAS goes low and before CAS goes low and held for the prescribed time.

			PORTC = (uint8_t)addr & 0b00111111; // only 5 pins on PORTC

			PORTB &= ~((1 << PB4) | (1 << PB5)); // clear bits again

			PORTB |= (uint8_t)(addr >> 2) & 0b00110000; // rest of address pins

			// WE must be set low for a certain time for a write operation to occur. The timing of the transitions are determined by CAS going low.

			// Data must be applied to the data input pins the prescribed amount of time before CAS goes low  and held.

			PORTB |= ((Dst[i] & 0b00001111) << 4);

			// CAS must switch from high to low.

			CAS_LO;

			// Only CAS goes HI for fpm mode

			CAS_HI;

			addr++;

			// clear outputs and adress pins

			PORTB &= ~(0b00111111);

			// The row address must be applied to the address input pins on the memory device for the prescribed amount of time before RAS goes low and be held for a period of time.

			PORTC = (uint8_t)(addr >> 8) & 0b00111111; // only 5 pins on PORTC
			PORTB |= (uint8_t)(addr >> 10) & 0b00110000; // rest of address pins

			// RAS must go from high to low.

			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time after RAS goes low and before CAS goes low and held for the prescribed time.

			PORTC = (uint8_t)addr & 0b00111111; // only 5 pins on PORTC

			PORTB &= ~((1 << PB4) | (1 << PB5)); // clear bits again

			PORTB |= (uint8_t)(addr >> 2) & 0b00110000; // rest of address pins

			// WE must be set low for a certain time for a write operation to occur. The timing of the transitions are determined by CAS going low.

			// Data must be applied to the data input pins the prescribed amount of time before CAS goes low  and held.

			PORTB |= Dst[i] & 0b00001111;

			// CAS must switch from high to low.

			CAS_LO;

			// Only CAS goes HI for fpm mode

			CAS_HI;

			addr++;
		}

		RAS_HI; // end of fpm mode
	}

}

void DramReadModifyWrite4b(uint16_t addr, uint8_t(*ModifyFunc)(uint8_t))
{
	uint8_t tmp;

	DDRB &= ~(0b00001111); // 4 bit input
	PORTB &= ~((1 << PB4) | (1 << PB5)); // clear adress pins to write new value

	WE_HI;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// The row address must be applied to the address input pins on the memory device for the prescribed
		// amount of time before RAS goes low and held after RAS goes low.

		PORTC = (uint8_t)(addr >> 8) & 0b00111111; // only 5 pins on PORTC
		PORTB |= (uint8_t)(addr >> 10) & 0b00110000; // rest of address pins

		// RAS must go from high to low and remain low.

		RAS_LO;

		// A column address must be applied to the address input pins on the memory device for the prescribed amount of time and held after CAS goes low.

		PORTC = (uint8_t)addr & 0b00111111; // only 5 pins on PORTC

		PORTB &= ~((1 << PB4) | (1 << PB5)); // clear bits again

		PORTB |= (uint8_t)(addr >> 2) & 0b00110000; // rest of address pins

		// WE must be set high for a read operation to occur prior to the transition of CAS, and remain high after the transition of CAS.

		// CAS must switch from high to low and remain low.

		CAS_LO;

		// OE goes low within the prescribed window of time.

		OE_LO;

		// Data appears at the data output pins of the memory device. The time at which the data appears depends on when RAS , CAS and OE went low, and when the address is supplied.

		//_delay_us(0.120); // possible bug point
		asm volatile("nop"::);

		tmp = PINB & 0b00001111; // 4 bit memory

		// OE must return to high state

		OE_HI;

		// Modify our data in by callback and write it back to memory

		DDRB |= 0b00001111; // 4 bit output
		PORTB &= ~(0b00001111); // clear outputs

		PORTB |= (ModifyFunc(tmp) & 0b00001111);

		// WE must be set low for a certain time for a write operation to occur

		WE_LO;

		//WE_HI;

		// Before the read-modify-write cycle can be considered complete, CAS and RAS must return to their inactive states.

		CAS_HI;
		RAS_HI;
	}

}

ISR(TIMER0_OVF_vect) // cas before ras refresh 	// 256 cycles
{
	LED1_ON;

	uint8_t i = 0xff;
	//for (uint16_t i=0 ; i < 256; i++)
	do
	{
		CAS_LO;		// CAS lo
					//Tcsr 10ns
		RAS_LO;		// RAS lo

		//_delay_us(0.120); // RAS cycle 120 ns.

		CAS_HI;		// CAS hi
		RAS_HI;	// RAS hi
				//asm volatile("nop" ::); // Trp 90ns
	} while (i--);

	LED1_OFF;
}