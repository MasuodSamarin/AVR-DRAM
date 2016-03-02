#include <avr/io.h> 
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "dram.h"


void RefreshTimerInt(void)
{
	// timer 0 overflow used for example
	// refresh peroid have to match with datasheet of the used memory (4-64 ms)
	
	TCCR0B |= (1<<CS02); // 256 // 3.5 ms refresh period at 18.432 MHz // 8ms at 8MHz
	TIMSK0 |= (1<<TOIE0); // overflow
}

void MemoryInit(void)
{
	___DDR(DATA_PORT) = 0xff;
	
#if defined(DRAM_LARGE_MEMORY_MODE) && defined(DRAM_HIGH_ADDRESS_PINS_ON_ANOTHER_PORT)
	___DDR(HIGH_ADDRESS_PORT) |= ((1<<(DRAM_ADDRESS_PINS-8)) - 1); 
#endif
	
	// can be merged to one call if using same port 
	___DDR(RAS_PORT) |= (1<<RAS_PIN);
	___DDR(CAS_PORT) |= (1<<CAS_PIN);
	___DDR(WE_PORT) |= (1<<WE_PIN);
	___DDR(OE_PORT) |= (1<<OE_PIN);
	___DDR(LA1_PORT) |= (1<<LA1_PIN);
	
#ifndef DRAM_HIGH_ADDRESS_PINS_ON_ANOTHER_PORT
	___DDR(LA2_PORT) |= (1<<LA2_PIN);
#endif
	
	CAS_HI;
	RAS_HI;
	
	uint8_t i = (DRAM_INIT_SEQUENCE_CYCLES - 1);
	do
	{
		CAS_LO;		// CAS lo
		RAS_LO;		// RAS lo
		
		//asm volatile("nop"::);
			
		CAS_HI;		// CAS hi
		RAS_HI;	// RAS hi
	} while(i--);
	
}

#ifdef DRAM_LARGE_MEMORY_MODE

	uint8_t DramRead(uint32_t addr)
	{
		uint8_t tmp;
		
		WE_HI;
		OE_LO;
		
		LA1_HI;
		
	#if !defined(DRAM_HIGH_ADDRESS_PINS_ON_ANOTHER_PORT)
		LA2_HI;
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			// The row address must be applied to the address input pins on the memory device for the prescribed
			// amount of time before RAS goes low and held after RAS goes low.
			
		#if defined(DRAM_HIGH_ADDRESS_PINS_ON_ANOTHER_PORT)
			
			___PORT(HIGH_ADDRESS_PORT) |= ((addr >> (DRAM_ADDRESS_PINS + 8)) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			___PORT(DATA_PORT) = (addr >> DRAM_ADDRESS_PINS);
			
		#else
			___PORT(DATA_PORT) = ((addr >> (DRAM_ADDRESS_PINS + 8)) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			
			LA2_LO;
			
			___PORT(DATA_PORT) = (addr >> DRAM_ADDRESS_PINS);
		#endif
			
			// RAS must go from high to low and remain low.
			
			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time and held after CAS goes low.

		#if defined(DRAM_HIGH_ADDRESS_PINS_ON_ANOTHER_PORT)
			
			___PORT(HIGH_ADDRESS_PORT) |= ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			___PORT(DATA_PORT) = addr;
			
		#else
			LA2_HI;
		
			___PORT(DATA_PORT) = ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			
			LA2_LO;
			
			___PORT(DATA_PORT) = addr;
		#endif
			
			LA1_LO; // lock cas address
			
			___DDR(DATA_PORT) = 0x00;
			
		#ifdef DRAM_CLEAR_PULLUPS_BEFORE_READING
			___PORT(DATA_PORT) = 0x00;
		#endif
			
			// CAS must switch from high to low and remain low.

			CAS_LO;

			// Data appears at the data output pins of the memory device. The time at which the data appears depends on when RAS , CAS and OE went low, and when the address is supplied.
			
			asm volatile("nop"::); // absolute delay due to 1,5 input read cycle delay
			DramDelayHook();
			
			tmp = ___PIN(DATA_PORT);
			
			// Before the read cycle can be considered complete, CAS and RAS must return to their inactive states.

			CAS_HI;
			RAS_HI;
		}
		
		___DDR(DATA_PORT) = 0xff; // set back to output state
		
		return tmp;
	}
	
	void DramWrite(uint32_t addr, uint8_t dat)
	{
		OE_HI;
		WE_LO; 
	
		LA1_HI;
		
	#if !defined(DRAM_HIGH_ADDRESS_PINS_ON_ANOTHER_PORT)
		LA2_HI;
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			// The row address must be applied to the address input pins on the memory device for the prescribed amount of time before RAS goes low and be held for a period of time.
		
		#if defined(DRAM_HIGH_ADDRESS_PINS_ON_ANOTHER_PORT)
			
			___PORT(HIGH_ADDRESS_PORT) |= ((addr >> DRAM_ADDRESS_PINS+8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			___PORT(DATA_PORT) = (addr >> DRAM_ADDRESS_PINS);
			
		#else
			___PORT(DATA_PORT) = ((addr >> (DRAM_ADDRESS_PINS + 8)) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			
			LA2_LO;
			
			___PORT(DATA_PORT) = (addr >> DRAM_ADDRESS_PINS);
		#endif
		
			// RAS must go from high to low.

			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time after RAS goes low and before CAS goes low and held for the prescribed time.
		
		#if defined(DRAM_HIGH_ADDRESS_PINS_ON_ANOTHER_PORT)
			
			___PORT(HIGH_ADDRESS_PORT) |= ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			___PORT(DATA_PORT) = addr;
			
		#else
			LA2_HI;
			
			___PORT(DATA_PORT) = ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			
			LA2_LO;
			
			___PORT(DATA_PORT) = addr;
		#endif
			
			LA1_LO; // lock cas address
		
			// Data must be applied to the data input pins the prescribed amount of time before CAS goes low  and held.
		
			___PORT(DATA_PORT) = dat;
		
			// CAS must switch from high to low.

			CAS_LO;
		
			DramDelayHook();

			// Before the write cycle can be considered complete, CAS and RAS must return to their inactive states.
		
			CAS_HI;
			RAS_HI;
		}
		
	}

#else // 64k memory
	
	uint8_t DramRead(uint16_t addr)
	{
		uint8_t tmp;
	
		WE_HI;
		OE_LO;
	
		LA1_HI;
	
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			// The row address must be applied to the address input pins on the memory device for the prescribed
			// amount of time before RAS goes low and held after RAS goes low.
		
			___PORT(DATA_PORT) = (addr >> 8);
		
			// RAS must go from high to low and remain low.
		
			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time and held after CAS goes low.

			___PORT(DATA_PORT) = (uint8_t)addr;
			LA1_LO; // lock cas address
		
			___DDR(DATA_PORT) = 0x00;
		
		#ifdef DRAM_CLEAR_PULLUPS_BEFORE_READING
			___PORT(DATA_PORT) = 0x00;
		#endif
		
			// CAS must switch from high to low and remain low.

			CAS_LO;

			// Data appears at the data output pins of the memory device. The time at which the data appears depends on when RAS , CAS and OE went low, and when the address is supplied.
		
			asm volatile("nop"::); // absolute delay due to 1,5 input read cycle delay
			DramDelayHook();
		
			tmp = ___PIN(DATA_PORT); 
		
			// Before the read cycle can be considered complete, CAS and RAS must return to their inactive states.

			CAS_HI;
			RAS_HI;
		}
	
		___DDR(DATA_PORT) = 0xff; // set back to output state
		
		return tmp;
	}

	void DramWrite(uint16_t addr, uint8_t dat)
	{
		OE_HI;
		WE_LO; 
	
		LA1_HI;
	
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			// The row address must be applied to the address input pins on the memory device for the prescribed amount of time before RAS goes low and be held for a period of time.
		
			___PORT(DATA_PORT) = (addr >> 8);
		
			// RAS must go from high to low.

			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time after RAS goes low and before CAS goes low and held for the prescribed time.
		
			___PORT(DATA_PORT) = (uint8_t)addr;
			LA1_LO; // lock cas address
		
			// Data must be applied to the data input pins the prescribed amount of time before CAS goes low  and held.
		
			___PORT(DATA_PORT) = dat;
		
			// CAS must switch from high to low.

			CAS_LO;
		
			DramDelayHook();

			// Before the write cycle can be considered complete, CAS and RAS must return to their inactive states.
		
			CAS_HI;
			RAS_HI;
		}
	}
#endif // !DRAM_LARGE_MEMORY_MODE

ISR(DRAM_REFRESH_INTERRUPT) // CAS before RAS refresh
{
#if DRAM_REFRESH_CYCLES <= 256
	uint8_t i = (DRAM_REFRESH_CYCLES - 1);
#else
	uint16_t i = (DRAM_REFRESH_CYCLES - 1);
#endif
	do
	{
	#ifdef DRAM_FAST_TOGGLE
		CAS_FAST_TOG;
		RAS_FAST_TOG;
	
		DramDelayHook();
	
		CAS_FAST_TOG;
		RAS_FAST_TOG;
	#else
		CAS_LO; 	// CAS lo
		RAS_LO; 	// RAS lo
		
		DramDelayHook();
		
		CAS_HI; 	// CAS hi
		RAS_HI; 	// RAS hi
	#endif
	} while(i--);
}