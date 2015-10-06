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

uint8_t PLATFORM_MCUSR = 0;

#include <avr/wdt.h>
// Disable watchdog or enable with 8 second timeout if WDTON fuse is programmed.
// Disabling would result in a 16ms watchdog if WDTON is programmed and user applications might not initialize fast enough.
#define platform_bootstrap() { \
    PLATFORM_MCUSR = MCUSR; \
    MCUSR = 0; \
	wdt_reset(); \
    wdt_disable(); \
    wdt_reset(); \
    WDTCSR |= (1<<WDCE) | (1<<WDE); \
    WDTCSR = (1<<WDP3) | (1<<WDP0); \
}

// Defines missing for RFR2 in avr-toolchain
#define PA_BUF_LT_6US 3
#define PA_LT_2US 0

// ---- From iom128rfa1.h ----

#define CMD_NOP                         0
#define CMD_TX_START                    2
#define CMD_FORCE_TRX_OFF               3
#define CMD_FORCE_PLL_ON                4
#define CMD_RX_ON                       6
#define CMD_TRX_OFF                     8
#define CMD_PLL_ON                      9
#define CMD_RX_AACK_ON                  22
#define CMD_TX_ARET_ON                  25
#define TRAC_SUCCESS                    0
#define TRAC_SUCCESS_DATA_PENDING       1
#define TRAC_SUCCESS_WAIT_FOR_ACK       2
#define TRAC_CHANNEL_ACCESS_FAILURE     3
#define TRAC_NO_ACK                     5
#define TRAC_INVALID                    7

// ----                   ----

#define P_ON                            0
#define BUSY_RX                         1
#define BUSY_TX                         2
#define RX_ON                           6
#define TRX_OFF                         8
#define PLL_ON                          9
#define SLEEP                           15
#define BUSY_RX_AACK                    17
#define BUSY_TX_ARET                    18
#define RX_AACK_ON                      22
#define TX_ARET_ON                      25
#define STATE_TRANSITION_IN_PROGRESS    31
#define TST_DISABLED                    0
#define TST_ENABLED                     1
#define CCA_BUSY                        0
#define CCA_IDLE                        1
#define CCA_NOT_FIN                     0
#define CCA_FIN                         1

// ----                   ----

#define PA_LT_2US                       0
#define PA_LT_4US                       1
#define PA_LT_6US                       2
#define PA_LT_8US                       3
#define PA_BUF_LT_0US                   0
#define PA_BUF_LT_2US                   1
#define PA_BUF_LT_4US                   2
#define PA_BUF_LT_6US                   3

// ----                   ----
#define CCA_CS                          2

// ---- From iom128rfa1.h ----

#endif
