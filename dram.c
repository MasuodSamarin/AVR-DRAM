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
#ifdef DRAM_SEPARATE_L_ADDR
	//___DDR(DATA_PORT) = 0x00;
#else
	___DDR(DATA_PORT) = 0xff;
#endif
	
#if defined(DRAM_LARGE_MEMORY_MODE) && defined(DRAM_SEPARATE_H_ADDR)
	___DDR(ADDRH_PORT) |= ((1<<(DRAM_ADDRESS_PINS-8)) - 1); 
#endif
	
	// can be merged into one call if using same port 
	___DDR(RAS_PORT) |= (1<<RAS_PIN);
	___DDR(CAS_PORT) |= (1<<CAS_PIN);
	___DDR(WE_PORT) |= (1<<WE_PIN);
	___DDR(OE_PORT) |= (1<<OE_PIN);
	
#ifndef DRAM_SEPARATE_L_ADDR
	___DDR(LA1_PORT) |= (1<<LA1_PIN);
#endif
	
#ifndef DRAM_SEPARATE_H_ADDR
	___DDR(LA2_PORT) |= (1<<LA2_PIN);
#endif
	
	CAS_HI;
	RAS_HI;
	
	uint8_t i = (DRAM_INIT_SEQUENCE_CYCLES - 1);
	do
	{
		CAS_LO;		// CAS lo
		RAS_LO;		// RAS lo
		
		DramDelayHook();
			
		CAS_HI;		// CAS hi
		RAS_HI;	// RAS hi
	} while(i--);
	
}

#ifdef DRAM_LARGE_MEMORY_MODE

	uint8_t DramDirectRead(uint16_t row, uint16_t column)
	{
		uint8_t tmp;
		
		WE_HI;
		OE_LO;
		
		LA1_HI;
		LA2_HI;
	
	#ifdef DRAM_SEPARATE_H_ADDR
		register uint8_t rtmp;
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
		#if defined(DRAM_SEPARATE_H_ADDR)
			rtmp = ___PORT(ADDRH_PORT); // save port content
			rtmp &= ~( (1<<(DRAM_ADDRESS_PINS-8)) - 1);
			___PORT(ADDRH_PORT) = rtmp | ((row >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			___PORT(ADDRH_PORT) = (row >> 8); // & ( (1<<(DRAM_ADDRESS_PINS-8)) - 1);
			LA2_LO;
		#endif
			
			___PORT(ADDRL_PORT) = (uint8_t)row;

			RAS_LO;

		#if defined(DRAM_SEPARATE_H_ADDR)
			___PORT(ADDRH_PORT) = rtmp | ((column >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			LA2_HI;
			___PORT(ADDRH_PORT) = (column >> 8); // & ( (1<<(DRAM_ADDRESS_PINS-8)) - 1);
			LA2_LO;
		#endif
		
			___PORT(ADDRL_PORT) = (uint8_t)column;
			LA1_LO; // lock cas address
			
			___DDR(DATA_PORT) = 0x00; //set port to input
			
			CAS_LO;
			
			___PORT(DATA_PORT) = 0x00; // clear pullups for the next latch-up and give 1 additional delay cycle
			DramDelayHook();
			
		#ifndef DRAM_EDO_MODE
			asm volatile("nop"::); // valid data have to appear on the port before half of the last delay clock cycle - strongly Tcac dependent
			tmp = ___PIN(DATA_PORT);
		#endif

			CAS_HI;
		#ifdef DRAM_EDO_MODE
			tmp = ___PIN(DATA_PORT);
		#endif
			
			RAS_HI;
		}

		___DDR(DATA_PORT) = 0xff; // set back to output state
		
		return tmp;
	}
	
	void DramDirectWrite(uint16_t row, uint16_t column, uint8_t dat)
	{
		OE_HI;
		WE_LO; 
	
		LA1_HI;
		LA2_HI;
		
	#ifdef DRAM_SEPARATE_H_ADDR
		register uint8_t rtmp;
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
		#if defined(DRAM_SEPARATE_H_ADDR)
			rtmp = ___PORT(ADDRH_PORT);
			rtmp &= ~( (1<<(DRAM_ADDRESS_PINS-8)) - 1);
			___PORT(ADDRH_PORT) = rtmp | ((row >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			___PORT(ADDRH_PORT) = (row >> 8); // & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			LA2_LO;
		#endif
		
			___PORT(ADDRL_PORT) = (uint8_t)row;
		
			RAS_LO;

		#if defined(DRAM_SEPARATE_H_ADDR)
			___PORT(ADDRH_PORT) = rtmp | ((column >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			LA2_HI;
			___PORT(ADDRH_PORT) = (column >> 8); // & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			LA2_LO;
		#endif
			
			___PORT(ADDRL_PORT) = (uint8_t)column;
			LA1_LO; // lock cas address
		
			___PORT(DATA_PORT) = dat;

			CAS_LO;
		
			DramDelayHook();

			CAS_HI;
			RAS_HI;
		}
		
	}

	uint8_t DramRead(uint32_t addr)
	{
		uint8_t tmp;
		
		WE_HI;
		OE_LO;
		
		LA1_HI;
		LA2_HI;
		
	#ifdef DRAM_SEPARATE_H_ADDR
		register uint8_t rtmp;
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			// The row address must be applied to the address input pins on the memory device for the prescribed
			// amount of time before RAS goes low and held after RAS goes low.
			
		#if defined(DRAM_SEPARATE_H_ADDR)
			rtmp = ___PORT(ADDRH_PORT);
			rtmp &= ~( (1<<(DRAM_ADDRESS_PINS-8)) - 1);
			___PORT(ADDRH_PORT) = rtmp | ((addr >> (DRAM_ADDRESS_PINS+8)) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			___PORT(ADDRH_PORT) = (addr >> (DRAM_ADDRESS_PINS + 8)); // & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			LA2_LO;
		#endif
			
			___PORT(ADDRL_PORT) = (addr >> DRAM_ADDRESS_PINS);
			
			// RAS must go from high to low and remain low.
			
			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time and held after CAS goes low.

		#if defined(DRAM_SEPARATE_H_ADDR)
			___PORT(ADDRH_PORT) = rtmp | ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			LA2_HI;
				___PORT(ADDRH_PORT) = (addr >> 8); // & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			LA2_LO;
		#endif
			
			___PORT(ADDRL_PORT) = addr;
			LA1_LO; // lock cas address
			
			___DDR(DATA_PORT) = 0x00; // set port to input
			
			// CAS must switch from high to low and remain low.

			CAS_LO;

			// Data appears at the data output pins of the memory device. The time at which the data appears depends on when RAS , CAS and OE went low, and when the address is supplied.
			
			___PORT(DATA_PORT) = 0x00; //clear pullups for the next latch-up and give 1 additional delay cycle
			
			DramDelayHook();
			
		#ifndef DRAM_EDO_MODE
			asm volatile("nop"::); // valid data have to appear on the port before half of the last delay clock cycle - strongly Tcac dependent
			tmp = ___PIN(DATA_PORT);
		#endif
			
			// Before the read cycle can be considered complete, CAS and RAS must return to their inactive states.

			CAS_HI;
		#ifdef DRAM_EDO_MODE
			tmp = ___PIN(DATA_PORT);
		#endif
			
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
		LA2_HI;
		
	#ifdef DRAM_SEPARATE_H_ADDR
		register uint8_t rtmp;
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			// The row address must be applied to the address input pins on the memory device for the prescribed amount of time before RAS goes low and be held for a period of time.
		
		#if defined(DRAM_SEPARATE_H_ADDR)
			rtmp = ___PORT(ADDRH_PORT);
			rtmp &= ~( (1<<(DRAM_ADDRESS_PINS-8)) - 1);
			___PORT(ADDRH_PORT) = rtmp | ((addr >> (DRAM_ADDRESS_PINS+8)) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			___PORT(DATA_PORT) = ((addr >> (DRAM_ADDRESS_PINS + 8)) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			LA2_LO;
		#endif
			
			___PORT(ADDRL_PORT) = (addr >> DRAM_ADDRESS_PINS);
		
			// RAS must go from high to low.

			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time after RAS goes low and before CAS goes low and held for the prescribed time.
		
		#if defined(DRAM_SEPARATE_H_ADDR)
			___PORT(ADDRH_PORT) = rtmp | ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			LA2_HI;
			___PORT(ADDRH_PORT) = ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			LA2_LO;
		#endif
			
			___PORT(ADDRL_PORT) = addr;
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

	void DramDirectPageRead(uint16_t row, uint16_t column, uint16_t count, uint8_t *Dst)
	{
		WE_HI;
		OE_LO;
		
		LA1_HI;
		LA2_HI;
		
	#ifdef DRAM_SEPARATE_H_ADDR
		register uint8_t rtmp;
	#endif
	
	#ifdef DRAM_SEPARATE_L_ADDR
		___DDR(DATA_PORT) = 0x00;
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
		#if defined(DRAM_SEPARATE_H_ADDR)
			rtmp = ___PORT(ADDRH_PORT);
			rtmp &= ~( (1<<(DRAM_ADDRESS_PINS-8)) - 1);
			___PORT(ADDRH_PORT) = rtmp | ((row >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			___PORT(ADDRH_PORT) = (row >> 8); // & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			LA2_LO;
		#endif
		
			___PORT(ADDRL_PORT) = (uint8_t)row;
			
			RAS_LO;
			
			for(uint16_t i = 0; i < count; i++)
			{
				LA1_HI;
			#if defined(DRAM_SEPARATE_H_ADDR)
				___PORT(ADDRH_PORT) = rtmp | ((column >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			#else
				LA2_HI;
				___PORT(ADDRH_PORT) = (column >> 8); // & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
				LA2_LO;
			#endif
				
				___PORT(ADDRL_PORT) = (uint8_t)column++;
				LA1_LO; // lock cas address
			
			#ifndef DRAM_SEPARATE_L_ADDR
				___DDR(DATA_PORT) = 0x00;
			#endif
				
				CAS_FAST_TOG_L;
				
				___PORT(DATA_PORT) = 0x00; //clear pullups for the next latch-up and give 1 additional delay cycle
				DramDelayHook();
			
			#ifndef DRAM_EDO_MODE
				asm volatile("nop"::); // valid data have to appear on the port before half of the last delay clock cycle - strongly Tcac dependent
				Dst[i] = ___PIN(DATA_PORT);
			#endif
				
				CAS_FAST_TOG_H;
			
			#ifdef DRAM_EDO_MODE
				Dst[i] = ___PIN(DATA_PORT);
			#endif
				
			#ifndef DRAM_SEPARATE_L_ADDR
				___DDR(DATA_PORT) = 0xff; // set back to output
			#endif
			} 
			
			RAS_HI;
			
		#ifdef DRAM_SEPARATE_L_ADDR
			___DDR(DATA_PORT) = 0xff; // set back to output
		#endif
		}
		
	}
	
	void DramDirectPageWrite(uint16_t row, uint16_t column, uint16_t count, uint8_t *Dst)
	{
		OE_HI;
		WE_LO;
		
		LA1_HI;
		LA2_HI;
		
	#ifdef DRAM_SEPARATE_H_ADDR
		register uint8_t rtmp;
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
		#if defined(DRAM_SEPARATE_H_ADDR)
			rtmp = ___PORT(ADDRH_PORT);
			rtmp &= ~( (1<<(DRAM_ADDRESS_PINS-8)) - 1);
			___PORT(ADDRH_PORT) = rtmp | ((row >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			___PORT(ADDRH_PORT) = (row >> 8); // & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			LA2_LO;
		#endif
			___PORT(ADDRL_PORT) = (uint8_t)row;
			
			RAS_LO;
			
			for(uint16_t i = 0; i < count; i++)
			{
				LA1_HI;
			#if defined(DRAM_SEPARATE_H_ADDR)
				___PORT(ADDRH_PORT) = rtmp | ((column >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			#else
				LA2_HI;
				___PORT(ADDRH_PORT) = (column >> 8); // & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
				LA2_LO;
			#endif
				
				___PORT(ADDRL_PORT) = (uint8_t)column++;
				LA1_LO; // lock cas address
				
				___PORT(DATA_PORT) = Dst[i]; 
				
				CAS_FAST_TOG_L;
				
				DramDelayHook();
				
				CAS_FAST_TOG_H;
			} 
			
			RAS_HI;
		}
	}

	void DramPageRead(uint32_t addr, uint16_t count, uint8_t *Dst)
	{
		WE_HI;
		OE_LO;
		
		LA1_HI;
		LA2_HI;
		
	#ifdef DRAM_SEPARATE_H_ADDR
		register uint8_t rtmp;
	#endif
	
	#ifdef DRAM_SEPARATE_L_ADDR
		___DDR(DATA_PORT) = 0x00; // set to input
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
		#if defined(DRAM_SEPARATE_H_ADDR)
			rtmp = ___PORT(ADDRH_PORT);
			rtmp &= ~( (1<<(DRAM_ADDRESS_PINS-8)) - 1);
			___PORT(ADDRH_PORT) = rtmp | ((addr >> (DRAM_ADDRESS_PINS+8)) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			___PORT(ADDRH_PORT) = (addr >> (DRAM_ADDRESS_PINS+8)); // & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			LA2_LO;
		#endif
			
			___PORT(ADDRL_PORT) = (addr >> DRAM_ADDRESS_PINS);
			
			RAS_LO;
			
			for(uint16_t i = 0; i < count; i++)
			{
				LA1_HI;
			#if defined(DRAM_SEPARATE_H_ADDR)
				___PORT(ADDRH_PORT) = rtmp | ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			#else
				LA2_HI;
				___PORT(ADDRH_PORT) = ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
				LA2_LO;
			#endif
				
				___PORT(ADDRL_PORT) = (uint8_t)addr++;
				LA1_LO; // lock cas address
				
			#ifndef DRAM_SEPARATE_L_ADDR
				___DDR(DATA_PORT) = 0x00; // set to input
			#endif
				
				CAS_FAST_TOG_L;
				
				___PORT(DATA_PORT) = 0x00; //clear pullups for the next latch-up and give 1 additional delay cycle
				DramDelayHook();
				
			#ifndef DRAM_EDO_MODE
				asm volatile("nop"::); // valid data have to appear on the port before half of the last delay clock cycle - strongly Tcac dependent
				Dst[i] = ___PIN(DATA_PORT);
			#endif
			
				CAS_FAST_TOG_H;
			
			#ifdef DRAM_EDO_MODE
				Dst[i] = ___PIN(DATA_PORT);
			#endif
				
			#ifndef DRAM_SEPARATE_L_ADDR
				___DDR(DATA_PORT) = 0xff; // set back to output
			#endif
			}
			
			RAS_HI;
			
		#ifdef DRAM_SEPARATE_L_ADDR
			___DDR(DATA_PORT) = 0xff; // set back to output
		#endif
		}
		
	}
	
	void DramPageWrite(uint32_t addr, uint16_t count, uint8_t *Dst)
	{
		
		OE_HI;
		WE_LO;
		
		LA1_HI;
		LA2_HI;
		
	#ifdef DRAM_SEPARATE_H_ADDR
		register uint8_t rtmp;
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
		#if defined(DRAM_SEPARATE_H_ADDR)
			rtmp = ___PORT(ADDRH_PORT);
			rtmp &= ~( (1<<(DRAM_ADDRESS_PINS-8)) - 1);
			___PORT(ADDRH_PORT) = rtmp | ((addr >> (DRAM_ADDRESS_PINS+8)) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
		#else
			___PORT(ADDRH_PORT) = ((addr >> (DRAM_ADDRESS_PINS + 8)) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			LA2_LO;
		#endif
		
			___PORT(ADDRL_PORT) = (addr >> DRAM_ADDRESS_PINS);
			
			RAS_LO;
			
			for(uint16_t i = 0; i < count; i++)
			{
				LA1_HI;
			#if defined(DRAM_SEPARATE_H_ADDR)
				___PORT(ADDRH_PORT) = rtmp | ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
			#else
				LA2_HI;
				___PORT(ADDRH_PORT) = ((addr >> 8) & ((1<<(DRAM_ADDRESS_PINS-8)) - 1));
				LA2_LO;
			#endif
				
				___PORT(ADDRL_PORT) = (uint8_t)addr++;
				LA1_LO; // lock cas address
				
				___PORT(DATA_PORT) = Dst[i]; 
				
				CAS_LO;
				
				DramDelayHook();
				
				CAS_HI;
			} 
			
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
		
			___PORT(ADDRL_PORT) = (addr >> 8);
		
			// RAS must go from high to low and remain low.
		
			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time and held after CAS goes low.

			___PORT(ADDRL_PORT) = (uint8_t)addr;
			LA1_LO; // lock cas address
		
			___DDR(DATA_PORT) = 0x00; //set port to input

			// CAS must switch from high to low and remain low.

			CAS_LO;

			// Data appears at the data output pins of the memory device. The time at which the data appears depends on when RAS , CAS and OE went low, and when the address is supplied.
		
			___PORT(DATA_PORT) = 0x00; //clear pullups for the next latch-up and give 1 additional delay cycle
			DramDelayHook();
		
		#ifndef DRAM_EDO_MODE
			asm volatile("nop"::); // valid data have to appear on the port before half of the last delay clock cycle - strongly Tcac dependent 
			tmp = ___PIN(DATA_PORT); 
		#endif
			// Before the read cycle can be considered complete, CAS and RAS must return to their inactive states.

			CAS_HI;
		#ifdef DRAM_EDO_MODE
			tmp = ___PIN(DATA_PORT);
		#endif
			
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
		
			___PORT(ADDRL_PORT) = (addr >> 8);
		
			// RAS must go from high to low.

			RAS_LO;

			// A column address must be applied to the address input pins on the memory device for the prescribed amount of time after RAS goes low and before CAS goes low and held for the prescribed time.
		
			___PORT(ADDRL_PORT) = (uint8_t)addr;
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
	
	void _DramPageRead(uint16_t addr, uint8_t count, uint8_t *Dst)
	{
		WE_HI;
		OE_LO;
		
	#ifdef DRAM_SEPARATE_L_ADDR
		___DDR(DATA_PORT) = 0x00; //set port to input
	#endif
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			___PORT(ADDRL_PORT) = (addr >> 8);
			
			RAS_LO;
			
			uint8_t i = 0;
			
			//for(int i = 0; i < count + 1; i++) // current implementation
			do 
			{
				LA1_HI;
				___PORT(ADDRL_PORT) = (uint8_t)addr++;
				LA1_LO; // lock cas address
				
			#ifndef DRAM_SEPARATE_L_ADDR
				___DDR(DATA_PORT) = 0x00; //set port to input
			#endif
				
				CAS_FAST_TOG_L;
				
				___PORT(DATA_PORT) = 0x00; //clear pullups for the next latch-up and give 1 additional delay cycle
				DramDelayHook();
				
			#ifndef DRAM_EDO_MODE
				asm volatile("nop"::); // valid data have to appear on the port before half of the last delay clock cycle - strongly Tcac dependent
				Dst[i] = ___PIN(DATA_PORT);
			#endif
				
				CAS_FAST_TOG_H;
				
			#ifdef DRAM_EDO_MODE
				Dst[i] = ___PIN(DATA_PORT);
			#endif
				
			#ifndef DRAM_SEPARATE_L_ADDR
				___DDR(DATA_PORT) = 0xff; // set back to output
			#endif
			} while(i++ != count);
			
			RAS_HI;
			
		#ifdef DRAM_SEPARATE_L_ADDR
			___DDR(DATA_PORT) = 0xff; // set back to output
		#endif
		}
		
	}
	
	void _DramPageWrite(uint16_t addr, uint8_t count, uint8_t *Dst)
	{
		OE_HI;
		WE_LO;
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			___PORT(ADDRL_PORT) = (addr >> 8);
			
			RAS_LO;
			
			uint8_t i = 0;
			
			//for(int i = 0; i < count + 1; i++) // current implementation
			do 
			{
				LA1_HI;
				___PORT(ADDRL_PORT) = (uint8_t)addr++;
				LA1_LO; // lock cas address
				
				___PORT(DATA_PORT) = Dst[i]; // i++
				
				CAS_FAST_TOG_L;
				
				DramDelayHook();
				
				CAS_FAST_TOG_H;
			} while(i++ != count); // count--
			
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
		CAS_FAST_TOG_L;
		RAS_FAST_TOG_L;
	
		DramDelayHook();
	
		CAS_FAST_TOG_H;
		RAS_FAST_TOG_H;
	} while(i--);
}