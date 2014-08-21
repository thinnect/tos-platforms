// disable watchdog timer at startup (see AVR132: Using the Enhanced Watchdog Timer)
#ifndef PLATFORM_H
#define PLATFORM_H

#include <avr/wdt.h>
#include "TinyError.h"

#define AT45DB_FLASH_CHIP
#define DS2401_PIN PortE4
// Battery
#define BATTERY_REF_VOLTAGE 5400UL
#define BATTERY_REF_VOLTAGE_TAG ATM128_ADC_VREF_AVDD
#define BATTERY_ADC_CHANNEL ATM128_ADC_SNGL_ADC0
#define BATTERY_PIN PortB7

// Security defines
#define EIGHT_BIT_PROCESSOR
#define INLINE_ASM
#define TINYECC_AVR_ASM

// defines and asserts needed for CC1101 radio driver
#define ASSERT_CANT_HAPPEN 0xBEEF
#define ASSERT_NONE

inline void doAssert2(bool condition, uint16_t errorCode) __attribute__((C)) {}
inline void doAssertNot2(bool condition, uint16_t errorCode) __attribute__((C)) {}
inline void doAssertSuccess2(error_t error, uint16_t errorCode) __attribute__((C)) {}
inline void doAssertEquals2(uint32_t a, uint32_t b, uint16_t errorCode) __attribute__((C)) {}

#define ccassert(condition, output) doAssert2((condition), (output))
#define ccassertNot(condition, output) doAssertNot2((condition), (output))
#define ccassertSuccess(error, output) doAssertSuccess2((error), (output))
#define ccassertEquals(a, b, output) doAssertEquals2((a), (b), (output))

// Tossim uses nanodbg for message passing with simulation environment
#if !defined(TOSSIM_ENABLE_NANOMSG_DEBUG_CHANNEL)
	#define nanodbg(s, ...)
#endif

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
#endif
