#ifndef DRAM_H_
#define DRAM_H_

#define DRAM_ADDRESS_PINS 12 // 8 for 64kB memory, 12 for 16MB memory
#define DRAM_INIT_SEQUENCE_CYCLES 8 // check datasheet
//#define DRAM_REFRESH_CYCLES 256 // set if not equally 2^DRAM_ADDRESS_PINS // can also be set to 256 if inlined refresh cycles multiple times in ISR for faster operation

//#define DRAM_EDO_MODE

//#define DRAM_SEPARATE_L_ADDR // separate low address port from data port // required for EDO mode operation // no LA1 required
//#define DRAM_SEPARATE_H_ADDR // separate high address port (>64k memories) form L_ADDR  // no LA2 required

//#define DRAM_FORCE_SLOW_STROBES // force 2 cycle sbi/cbi signalling instead of 1 cycle PINx hardware xor // can fix some memory timming issues

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

#if defined(DRAM_EDO_MODE)&&!defined(DRAM_SEPARATE_L_ADDR)
	#warning "to prevent short circuits - do not use EDO memories with fpm read without separating address lines from data lines"
#endif

	#define RAS_PORT D // A,B,C,D ... port naming 
	#define RAS_PIN 0 // 1,2,3,4 ... pin naming

	#define CAS_PORT D // A,B,C,D ... port naming
	#define CAS_PIN 1 // 1,2,3,4 ... pin naming

	#define WE_PORT D // A,B,C,D ... port naming
	#define WE_PIN 2 // 1,2,3,4 ... pin naming

	#define OE_PORT D // A,B,C,D ... port naming
	#define OE_PIN 3 // 1,2,3,4 ... pin naming


	#define DATA_PORT B // A,B,C,D ... port naming // port used for data i/o and ADDRESSing

	#ifdef DRAM_SEPARATE_L_ADDR
		#define ADDRL_PORT A // A,B,C,D ... port naming
	#else
		#define ADDRL_PORT DATA_PORT 
	
		#define LA1_PORT D // A,B,C,D ... port naming
		#define LA1_PIN 4 // 1,2,3,4 ... pin naming
	#endif

	#ifdef DRAM_SEPARATE_H_ADDR
		#define ADDRH_PORT C // A,B,C,D ... port naming
	#else
		#define ADDRH_PORT ADDRL_PORT // A,B,C,D ... port naming
	
		#define LA2_PORT D // A,B,C,D ... port naming
		#define LA2_PIN 5 // 1,2,3,4 ... pin naming
	#endif

// add any valid mcu
#if defined(__AVR_ATmega48__)||defined(__AVR_ATmega48P__)||defined(__AVR_ATmega48PA__)||defined(__AVR_ATmega48PB__)\
||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88PB__)\
||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168P__)||defined(__AVR_ATmega168PA__)||defined(__AVR_ATmega168PB__)\
||defined(__AVR_ATmega328__)||defined(__AVR_ATmega328P__)||defined(__AVR_ATmega328PB__)
	#ifndef DRAM_FORCE_SLOW_STROBES
		#define DRAM_FAST_TOGGLE // allow fast xor'ing outputs by writing to PINn registers // used for burst sequences
	#endif
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

#ifdef DRAM_FAST_TOGGLE
	#define RAS_FAST_TOG_L ___PIN(RAS_PORT) = (1<<RAS_PIN)
	#define RAS_FAST_TOG_H ___PIN(RAS_PORT) = (1<<RAS_PIN)
#else
	#define RAS_FAST_TOG_L RAS_LO
	#define RAS_FAST_TOG_H RAS_HI
#endif

#define CAS_HI ___PORT(CAS_PORT) |= (1<<CAS_PIN)
#define CAS_LO ___PORT(CAS_PORT) &= ~(1<<CAS_PIN)

#ifdef DRAM_FAST_TOGGLE
	#define CAS_FAST_TOG_L ___PIN(CAS_PORT) = (1<<CAS_PIN)
	#define CAS_FAST_TOG_H ___PIN(CAS_PORT) = (1<<CAS_PIN)
#else
	#define CAS_FAST_TOG_L CAS_LO
	#define CAS_FAST_TOG_H CAS_HI
#endif

#define WE_HI ___PORT(WE_PORT) |= (1<<WE_PIN)
#define WE_LO ___PORT(WE_PORT) &= ~(1<<WE_PIN)
//#define WE_FAST_TOG ___PIN(WE_PORT) = (1<<WE_PIN)

#define OE_HI ___PORT(OE_PORT) |= (1<<OE_PIN)
#define OE_LO ___PORT(OE_PORT) &= ~(1<<OE_PIN)
//#define OE_FAST_TOG ___PIN(OE_PORT) = (1<<OE_PIN)

#ifdef DRAM_SEPARATE_L_ADDR
	#define LA1_HI ((void)0)
	#define LA1_LO ((void)0)
#else
	#define LA1_HI ___PORT(LA1_PORT) |= (1<<LA1_PIN)
	#define LA1_LO ___PORT(LA1_PORT) &= ~(1<<LA1_PIN)
	//#define LA1_FAST_TOG ___PIN(LA1_PORT) = (1<<LA1_PIN)
#endif

#ifdef DRAM_SEPARATE_H_ADDR
	#define LA2_HI ((void)0)
	#define LA2_LO ((void)0)
#else
	#define LA2_HI ___PORT(LA2_PORT) |= (1<<LA2_PIN)
	#define LA2_LO ___PORT(LA2_PORT) &= ~(1<<LA2_PIN)
	//#define LA2_FAST_TOG ___PIN(LA2_PORT) = (1<<LA2_PIN)
#endif

// for slower memories // delays are executed after CAS line goes low
// more correct timing with minimum overhead can be obtained by experimental 
// placing delays directly in the suspicious functions
static inline void DramDelayHook(void) __attribute__((always_inline));
static inline void DramDelayHook(void)
{
	//asm volatile("nop"::); // 1 cycle
	//asm volatile("rjmp .+0"::); // 2 cycles
}

void RefreshTimerInt(void);
void MemoryInit(void); // have to be called before enabling interrupts // Initialization sequence depends on datasheet of target memory

#ifdef DRAM_LARGE_MEMORY_MODE
	uint8_t DramDirectRead(uint16_t row, uint16_t column); // avoid expensive shift operations on address
	void DramDirectWrite(uint16_t row, uint16_t column, uint8_t dat); // avoid expensive shift operations on address
	
	//static inline uint8_t DramRead(uint32_t addr) __attribute__((always_inline));
	static inline uint8_t DramRead(uint32_t addr) { return DramDirectRead((addr << (8 - (DRAM_ADDRESS_PINS - 8)) >> 16), addr); }
	
	//static inline void DramWrite(uint32_t addr, uint8_t dat) __attribute__((always_inline));
	static inline void DramWrite(uint32_t addr, uint8_t dat) { DramDirectWrite((addr << (8 - (DRAM_ADDRESS_PINS - 8)) >> 16), addr, dat); }
	
	void DramDirectPageRead(uint16_t row, uint16_t column, uint16_t count, uint8_t *Dst);
	void DramDirectPageWrite(uint16_t row, uint16_t column, uint16_t count, uint8_t *Dst);
	
	//static inline void DramPageRead(uint32_t addr, uint16_t count, uint8_t *Dst) __attribute__((always_inline));
	static inline void DramPageRead(uint32_t addr, uint16_t count, uint8_t *Dst) { DramDirectPageRead((addr << (8 - (DRAM_ADDRESS_PINS - 8)) >> 16), addr, count, Dst); }
	
	//static inline void DramPageWrite(uint32_t addr, uint16_t count, uint8_t *Dst) __attribute__((always_inline));
	static inline void DramPageWrite(uint32_t addr, uint16_t count, uint8_t *Dst) { DramDirectPageWrite((addr << (8 - (DRAM_ADDRESS_PINS - 8)) >> 16), addr, count, Dst); } 
#else
	uint8_t DramRead(uint16_t addr);
	void DramWrite(uint16_t addr, uint8_t dat);
	
	void _DramPageRead(uint16_t addr, uint8_t count, uint8_t *Dst); // this function will read n + 1 bytes
	void _DramPageWrite(uint16_t addr, uint8_t count, uint8_t *Dst); // this function will write n + 1 bytes
	
	static inline void DramPageRead(uint16_t addr, uint16_t count, uint8_t *Dst) __attribute__((always_inline));
	static inline void DramPageRead(uint16_t addr, uint16_t count, uint8_t *Dst) { _DramPageRead(addr, count-1, Dst); }
		
	static inline void DramPageWrite(uint16_t addr, uint16_t count, uint8_t *Dst) __attribute__((always_inline));
	static inline void DramPageWrite(uint16_t addr, uint16_t count, uint8_t *Dst) { _DramPageWrite(addr, count-1, Dst); } 
#endif

#endif /* DRAM_H_ */