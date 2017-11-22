/**
 * The implementation of uS alarm using RAIL library.
 *
 * @author Konstantin Bilozor
 * @date   November 2017
 */

#include "rail.h"
#include "rail_types.h"
#include "rail_chip_specific.h"
#include "main.h"
#include "stdio.h"
#include "loglevels.h"


generic module AlarmRailMicroP(typedef precision) @safe() {
	provides {
		interface Init;
		interface Alarm<precision, uint32_t>;
		interface Counter<precision, uint32_t>;
	}
}

implementation {

	#define __MODUUL__ "AlarmRail"
	#define __LOG_LEVEL__ ( LOG_LEVEL_AlarmRail & BASE_LOG_LEVEL )
	#include "log.h"

	uint32_t t0, dt;

	void RAILCb_TimerExpired(RAIL_Handle_t railHandle) {
		signal Alarm.fired();
	}

	command error_t Init.init() {
		debug1("railHandle=%p", railHandle);
		
		return SUCCESS;
	}

	async command uint32_t Counter.get() {
		return RAIL_GetTime();
	}

	async command bool Counter.isOverflowPending() {
		return 0;
	}

	async command void Counter.clearOverflow() {

	}

	async command void Alarm.start(uint32_t ndt) {
		atomic call Alarm.startAt((uint32_t)RAIL_GetTime(), ndt);
	}

	async command void Alarm.stop() {
		RAIL_CancelTimer(railHandle);
	}

	async command bool Alarm.isRunning() {
		atomic 
		return RAIL_IsTimerRunning(railHandle);
	}

	async command void Alarm.startAt(uint32_t nt0, uint32_t ndt) {
		atomic {
			int32_t stat = 0;
			uint32_t now = 0;	
			now = (uint32_t)RAIL_GetTime();
			t0 = nt0;
			dt = ndt;

			ndt = nt0 + ndt - now;
			
			// warn1("start ACK alarm");
			stat = RAIL_SetTimer(railHandle, ndt, RAIL_TIME_ABSOLUTE, &RAILCb_TimerExpired);

			if (stat != 0) {
				debug1("failed start stat = %i", stat);
			}
		}
	}

	async command uint32_t Alarm.getNow() {
		atomic return RAIL_GetTime();
	}

	async command uint32_t Alarm.getAlarm() {
		
		atomic {
			debug1("getAlarm=%u, %u", t0+dt, (uint32_t)RAIL_GetTime());	
			return t0 + dt;
		}
	}
}

