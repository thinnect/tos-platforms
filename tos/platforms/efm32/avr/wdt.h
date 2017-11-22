/**
 * @author Raido Pahtma
 * @license MIT
 */
#ifndef _WDT_H_
#define _WDT_H_

#warning "TOSSIM ar wdt"

void wdt_reset();
void wdt_enable(uint16_t t);
void wdt_disable();

#define WDTO_15MS   0
#define WDTO_30MS   1
#define WDTO_60MS   2
#define WDTO_120MS  3
#define WDTO_250MS  4
#define WDTO_500MS  5
#define WDTO_1S     6
#define WDTO_2S     7
#define WDTO_4S     8
#define WDTO_8S     9

#endif // _WDT_H_
