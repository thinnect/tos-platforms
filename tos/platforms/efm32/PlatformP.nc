/**
 * Adapted from SAM3S_EK
 */

#include "hardware.h" // IRQ
#include "SignatureArea.h"
#include "SignatureAreaFile.h"
#include "DeviceSignature.h"

// uint16_t TOS_NODE_ID;

module PlatformP
{
    provides
	{
        interface Init;
    }
	uses
	{
		interface Init as LedsInit;
        interface Init as IRQInit;
        //interface Sam3LowPower;
	}
}

implementation
{
	#define __MODUUL__ "PlatformP"
	#define __LOG_LEVEL__ ( LOG_LEVEL_PlatformP & BASE_LOG_LEVEL )
	#include "log.h"

	command error_t Init.init()
	{
		/* I/O pin configuration, clock calibration, and LED configuration
		 * (see TEP 107)
		 */
		//call IRQInit.init();
		sigInit();
		TOS_NODE_ID = sigGetNodeId();
		debug1("%04X", TOS_NODE_ID);
		if ((TOS_NODE_ID == 0xFFFF) || (TOS_NODE_ID == 0)) {
			TOS_NODE_ID = 1;
			warn1("DEFAULT TOS_NODE_ID = 1");
		} else {
			debug1("TOS_NODE_ID=%u", TOS_NODE_ID);
		}
		//call LedsInit.init();

		return SUCCESS;
	}

    //async event void Sam3LowPower.customizePio() {
        // currently not optimized for sam3s-ek
   // }

	default command error_t LedsInit.init()
	{
		return SUCCESS;
	}
}
