#ifndef PLATFORM_H
#define PLATFORM_H

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

#include <avr/wdt.h>
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
