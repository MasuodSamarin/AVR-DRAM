#ifndef DRAM_H_
#define DRAM_H_

#define DRAM_ADDRESS_PINS 8 // eg. 12 for 16MB memory
#define DRAM_INIT_SEQUENCE_CYCLES 8 // check datasheet
//#define DRAM_REFRESH_CYCLES 256 // set if not equally 2^DRAM_ADDRESS_PINS // can also be set to 256 if inlined refresh cycles multiple times in ISR for faster operation

//#define DRAM_HIGH_ADDRESS_PORT // instead of second latch

#define DRAM_REFRESH_INTERRUPT TIMER0_OVF_vect // corresponding to timer initialized in RefreshTimerInt()

#ifndef DRAM_REFRESH_CYCLES
	#define DRAM_REFRESH_CYCLES (1UL<<DRAM_ADDRESS_PINS)
#endif

#if DRAM_ADDRESS_PINS < 8
	#error "<64kB memories not supported"
#elif DRAM_ADDRESS_PINS > 16
	#error ">4GB memories not supported"
#elif DRAM_ADDRESS_PINS != 8
	#define DRAM_LARGE_MEMORY_MODE
#endif

#ifdef DRAM_HIGH_ADDRESS_PORT
	#define HIGH_ADDRESS_PORT C // A,B,C,D ... port naming 
#endif

	#define DATA_PORT B // A,B,C,D ... port naming // port used for data i/o and ADDRESSing

	#define RAS_PORT D // A,B,C,D ... port naming 
	#define RAS_PIN 0 // 1,2,3,4 ... pin naming

	#define CAS_PORT D // A,B,C,D ... port naming
	#define CAS_PIN 1 // 1,2,3,4 ... pin naming

	#define WE_PORT D // A,B,C,D ... port naming
	#define WE_PIN 2 // 1,2,3,4 ... pin naming

	#define OE_PORT D // A,B,C,D ... port naming
	#define OE_PIN 3 // 1,2,3,4 ... pin naming

	#define LA1_PORT D // A,B,C,D ... port naming
	#define LA1_PIN 4 // 1,2,3,4 ... pin naming

#ifndef DRAM_HIGH_ADDRESS_PORT
	#define LA2_PORT D // A,B,C,D ... port naming
	#define LA2_PIN 5 // 1,2,3,4 ... pin naming
#endif

// add any valid mcu
#if defined(__AVR_ATmega48__)||defined(__AVR_ATmega48P__)||defined(__AVR_ATmega48PA__)||defined(__AVR_ATmega48PB__)\
||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88PB__)\
||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168P__)||defined(__AVR_ATmega168PA__)||defined(__AVR_ATmega168PB__)\
||defined(__AVR_ATmega328__)||defined(__AVR_ATmega328P__)||defined(__AVR_ATmega328PB__)
	#define DRAM_FAST_TOGGLE // allow fast xor'ing outputs by writing to PINn registers // for burst sequences
#endif


#ifndef ___DDR
	#define ___DDR(x) ___XDDR(x)
#endif
#ifndef ___XDDR
	#define ___XDDR(x) (DDR ## x)
#endif

#ifndef ___PORT
	#define ___PORT(x) ___XPORT(x)
#endif
#ifndef ___XPORT
	#define ___XPORT(x) (PORT ## x)
#endif

#ifndef ___PIN
	#define ___PIN(x) ___XPIN(x)
#endif
#ifndef ___XPIN
	#define ___XPIN(x) (PORT ## x)
#endif


#define RAS_HI ___PORT(RAS_PORT) |= (1<<RAS_PIN)
#define RAS_LO ___PORT(RAS_PORT) &= ~(1<<RAS_PIN)
#define RAS_FAST_TOG ___PIN(RAS_PORT) = (1<<RAS_PIN)

#define CAS_HI ___PORT(CAS_PORT) |= (1<<CAS_PIN)
#define CAS_LO ___PORT(CAS_PORT) &= ~(1<<CAS_PIN)
#define CAS_FAST_TOG ___PIN(CAS_PORT) = (1<<CAS_PIN)

#define WE_HI ___PORT(WE_PORT) |= (1<<WE_PIN)
#define WE_LO ___PORT(WE_PORT) &= ~(1<<WE_PIN)
#define WE_FAST_TOG ___PIN(WE_PORT) = (1<<WE_PIN)

#define OE_HI ___PORT(OE_PORT) |= (1<<OE_PIN)
#define OE_LO ___PORT(OE_PORT) &= ~(1<<OE_PIN)
#define OE_FAST_TOG ___PIN(OE_PORT) = (1<<OE_PIN)

#define LA1_HI ___PORT(LA1_PORT) |= (1<<LA1_PIN)
#define LA1_LO ___PORT(LA1_PORT) &= ~(1<<LA1_PIN)
#define LA1_FAST_TOG ___PIN(LA1_PORT) = (1<<LA1_PIN)

#ifndef DRAM_HIGH_ADDRESS_PORT
	#define LA2_HI ___PORT(LA2_PORT) |= (1<<LA2_PIN)
	#define LA2_LO ___PORT(LA2_PORT) &= ~(1<<LA2_PIN)
	#define LA2_FAST_TOG ___PIN(LA2_PORT) = (1<<LA2_PIN)
#endif


// for slower memories // delays are used in all parts of the code although only 
// one of the functions requires one cycle delay more correct timing with minimum 
// overhead can be obtained by experimental placing delays directly in the suspicious functions
inline void DramDelayHook(void)  
{
	//asm volatile("nop"::); // 1 cycle
	//asm volatile("rjmp .+0"::); // 2 cycles
}


void RefreshTimerInt(void);
void MemoryInit(void); 	// Initialization sequence depends on datasheet of target memory

#ifdef DRAM_LARGE_MEMORY_MODE
	uint8_t DramDirectRead(uint16_t row, uint16_t column); // avoid expensive shift operations on address
	void DramDirectWrite(uint16_t row, uint16_t column, uint8_t dat); // avoid expensive shift operations on address
	
	uint8_t DramRead(uint32_t addr);
	void DramWrite(uint32_t addr, uint8_t dat);
	
	void DramDirectPageRead(uint16_t row, uint16_t column, uint16_t count, uint8_t *Dst);
	void DramDirectPageWrite(uint16_t row, uint16_t column, uint16_t count, uint8_t *Dst);
	
	void DramPageRead(uint32_t addr, uint16_t count, uint8_t *Dst);
	void DramPageWrite(uint32_t addr, uint16_t count, uint8_t *Dst);
#else
	uint8_t DramRead(uint16_t addr);
	void DramWrite(uint16_t addr, uint8_t dat);
	
	void _DramPageRead(uint16_t addr, uint8_t count, uint8_t *Dst); // this function will read n + 1 bytes
	void _DramPageWrite(uint16_t addr, uint8_t count, uint8_t *Dst); // this function will write n + 1 bytes
	
	inline void DramPageRead(uint16_t addr, uint16_t count, uint8_t *Dst) { _DramPageRead(addr, count-1, Dst); }
	inline void DramPageWrite(uint16_t addr, uint16_t count, uint8_t *Dst) { _DramPageWrite(addr, count-1, Dst); } 
#endif


#endif /* DRAM_H_ */