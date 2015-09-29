/**
 * Internal flash parameters for the denode platform with Atmega128RFA1.
 *
 * @author Raido Pahtma
 */
#ifndef INTERNALFLASH_H_
#define INTERNALFLASH_H_

#include <avr/eeprom.h>

enum {
	EEPROM_SIZE = 0x1000,
	INT_FLASH_PAGE_SIZE = SPM_PAGESIZE,
};

#endif // _INTERNALFLASH_H_
