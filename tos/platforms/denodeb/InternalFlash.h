/**
 * Internal flash parameters for the denodeb platform with Atmega256RFR2.
 *
 * @author Raido Pahtma
 * @license MIT
 */
#ifndef INTERNALFLASH_H_
#define INTERNALFLASH_H_

#include <avr/eeprom.h>

#warning "denodeb EEPROM_SIZE at 0x1000 for compatibility"

enum {
	EEPROM_SIZE = 0x1000, // Actual size is 0x2000 for 256RFR2
	INT_FLASH_PAGE_SIZE = SPM_PAGESIZE,
};

#endif // _INTERNALFLASH_H_
