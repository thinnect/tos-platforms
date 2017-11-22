// $Id: Atm2560Alarm2P.nc,v 1.3 2010-06-29 22:07:43 scipio Exp $
/*
 * Copyright (c) 2007 Intel Corporation
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
#include <cmsis_os2.h>
#include "stdio.h"
#include "loglevels.h"


generic module efm32CmsisTimerP(typedef precision) @safe() {
	provides {
		interface Init;
		interface Alarm<precision, uint32_t>;
		interface Counter<precision, uint32_t>;
	}
}

implementation {

	#define __MODUUL__ "AlarmCnt"
	#define __LOG_LEVEL__ ( LOG_LEVEL_AlarmCnt & BASE_LOG_LEVEL )
	#include "log.h"

	uint32_t cnt = 0;
	uint8_t set = 0;
	uint32_t t0, dt;
	osTimerId_t oneShot_id;

	void timerEvent(void) {
		signal Alarm.fired();
	}

	command error_t Init.init() {
		__nesc_enable_interrupt();
		oneShot_id = osTimerNew((osTimerFunc_t)&timerEvent, osTimerOnce, NULL, NULL);
		debug1("oneShot_id=%p", oneShot_id);
		__nesc_disable_interrupt();
		
		return SUCCESS;
	}

	async command uint32_t Counter.get() {
		
		atomic
		{
			uint32_t now = 0;	
			now = (uint32_t)osKernelGetTickCountISR();
			return now;
		}
		
		
	}

	async command bool Counter.isOverflowPending() {
		return 0;
	}

	async command void Counter.clearOverflow() {

	}

	async command void Alarm.start(uint32_t ndt) {
		atomic call Alarm.startAt((uint32_t)osKernelGetTickCountISR(), ndt);
	}

	async command void Alarm.stop() {
		int32_t stat = 0;
		int32_t stat1 = 0;
		atomic {
			if (osTimerIsRunning(oneShot_id)) {
				stat = osTimerStopISR(oneShot_id);
			}
		}
	}

	async command bool Alarm.isRunning() {
		atomic 
		return osTimerIsRunning(oneShot_id);
	}

	async command void Alarm.startAt(uint32_t nt0, uint32_t ndt) {
		atomic {
			int32_t stat = 0;
			uint32_t now = 0;	
			now = (uint32_t)osKernelGetTickCountISR();
			t0 = nt0;
			dt = ndt;
			
			if (nt0+ndt < now) {
				ndt = 1;
				debug1("nt0+ndt < now");
			} else {
				ndt = nt0 + ndt - now;
			}
			
			stat = osTimerStartISR(oneShot_id, ndt);

			if (stat != 0) {
				debug1("failed start stat = %i", stat);
			}
		}
	}

	async command uint32_t Alarm.getNow() {
		atomic return call Counter.get();
	}

	async command uint32_t Alarm.getAlarm() {
		
		atomic {
			debug1("getAlarm=%u, %u", t0+dt, (uint32_t)osKernelGetTickCountISR());	
			return t0 + dt;
		}
	}
}

