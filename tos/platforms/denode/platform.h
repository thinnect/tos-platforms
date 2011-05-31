// disable watchdog timer at startup (see AVR132: Using the Enhanced Watchdog Timer)
#include <avr/wdt.h>

/* Pin Change Interrupt Vectors */
#define SIG_PIN_CHANGE0  _VECTOR(9)

/* Disable watchdog or enable with 8 second timeout if WDTON fuse is programmed. */
/* (Disabling would result in a 16ms watchdog if WDTON is programmed and user
 * applications might not initialize fast enough) */
#define platform_bootstrap() { \
    MCUSR &= ~(1<<WDRF); \
	wdt_reset(); \
    wdt_disable(); \
    wdt_reset(); \
    WDTCSR |= (1<<WDCE) | (1<<WDE); \
    WDTCSR = (1<<WDP3) | (1<<WDP0); \
}
