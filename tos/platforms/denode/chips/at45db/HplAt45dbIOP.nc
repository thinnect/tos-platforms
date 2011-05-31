// $Id: HplAt45dbIOP.nc,v 1.3 2010-06-29 22:07:53 scipio Exp $

/*
 * Copyright (c) 2000-2003 The Regents of the University  of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2002-2003 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */
/*
 * Copyright (c) 2007, Vanderbilt University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * Low level hardware access to the onboard AT45DB flash chip.
 * <p>
 * Note: This component includes optimised bit-banging SPI code with the
 * pins hardwired.  Don't copy it to some other platform without
 * understanding it (see txByte).
 *
 * @author Jason Hill
 * @author David Gay
 * @author Philip Levis
 * @author Janos Sallai <janos.sallai@vanderbilt.edu>
 */

#include "Timer.h"
#include "HplAt45db.h"
module HplAt45dbIOP {
  provides {
    interface Init;
    interface SpiByte as FlashSpi;
    interface HplAt45dbByte;
  }
  uses {
  	interface SpiByte;
    interface GeneralIO as Select;
    interface Resource;
//    interface BusyWait<TMicro, uint16_t>;
  }
}
implementation
{
  #define __MODUUL__ "ha45io"
  #define __LOG_LEVEL__ ( LOG_LEVEL_ha45io & BASE_LOG_LEVEL )
  #include "log.h"

  enum {
  	AT45_SR_RDY_BUSY = 1 << 7,
  	AT45_SR_COMP     = 1 << 6
  };

  command error_t Init.init() {
  	error_t err;
  	err = call Resource.immediateRequest();
  	logger(LOG_DEBUG2, "init(%u)", err);
  	// SPI init?
    call Select.makeOutput();
    call Select.set();
    return SUCCESS;
  }

  command void HplAt45dbByte.select() {
  	logger(LOG_DEBUG2, "select");
    call Select.clr();
  }

  command void HplAt45dbByte.deselect() {
  	logger(LOG_DEBUG2, "deselect");
    call Select.set();
  }

  async command uint8_t FlashSpi.write(uint8_t spiOut) {
  	uint8_t spiIn = 0;
//  	logger(LOG_DEBUG2, "w(%02x)", spiOut);
  	spiIn = call SpiByte.write(spiOut);
//  	logger(LOG_DEBUG2, "r(%02x)", spiIn);
  	return spiIn;
  }

  task void avail() {
    signal HplAt45dbByte.idle();
  }

  command void HplAt45dbByte.waitIdle() {
    uint8_t spiIn = 0;
    logger(LOG_DEBUG2, "wI");
    spiIn = call SpiByte.write(AT45_C_REQ_STATUS);
//    CLR_BIT(PORTD, 6);
    while(!(spiIn & AT45_SR_RDY_BUSY)) {
      spiIn = call SpiByte.write(AT45_C_REQ_STATUS);
    }
//    SET_BIT(PORTD, 6);
    post avail();
  }

  command bool HplAt45dbByte.getCompareStatus() {
  	logger(LOG_DEBUG2, "gCS");
    return !(AT45_SR_COMP & call SpiByte.write(AT45_C_REQ_STATUS));
  }
  
  event void Resource.granted() {
	
  }
  
}
