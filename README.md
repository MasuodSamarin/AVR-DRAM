#DRAM LIBRARY FOR AVR

- linear addresing
- requires only one port + 5-6 control pins and 8bit memory with CBR refresh mode.

#SETUP
Before setting up memory several conditions must be met:

- define DRAM_ADDRESS_PINS to value corresponding with used memory
- define DRAM_INIT_SEQUENCE_CYCLES to value corresponding to datasheet of used memory
- define all control pins and full 8 bit port for data operation 

#optional
- modify RefreshTimerInt() with corresponding DRAM_REFRESH_INTERRUPT macro for most optimal refresh peroid corresponding to datasheet
(example code uses timer 0 with 256 prescaler and overflow interrupt wchich should be safe but not optimal)
- if you prefer to have only one latch at a cost of another port (pins 0,1,2 up to DRAM_ADDRESS_PINS - 8) define DRAM_HIGH_ADDRESS_PINS_ON_ANOTHER_PORT macro
- if memory doesn't want to work correctly uncomment some nop's in DramDelayHook() inline function

#todo
- fpm r/w/rmw
- rmw
- code for testing memory
- DRAM_FAST_TOGGLE for more families than mega88