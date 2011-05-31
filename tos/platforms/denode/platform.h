// disable watchdog timer at startup (see AVR132: Using the Enhanced Watchdog Timer)
#include <avr/wdt.h>

/* Pin Change Interrupt Vectors */
#define SIG_PIN_CHANGE0  _VECTOR(9)
 
#define platform_bootstrap() { MCUSR = 0; wdt_disable(); }
