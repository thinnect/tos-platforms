/**
 * Denode AT45DB wiring.
 * @author Raido Pahtma
 * @license MIT
 */
#include "HplAt45db_chip.h"
configuration HplAt45dbC {
	provides interface HplAt45db @atmostonce();
}
implementation {

	// +1 because the AT45DB has 264/528 byte pages (log2 page size rounded up)
	components new HplAt45dbByteC(AT45_PAGE_SIZE_LOG2+1), HplAt45dbIOC;

	HplAt45db = HplAt45dbByteC;

	HplAt45dbByteC.Resource -> HplAt45dbIOC;
	HplAt45dbByteC.FlashSpi -> HplAt45dbIOC;
	HplAt45dbByteC.HplAt45dbByte -> HplAt45dbIOC;

}
