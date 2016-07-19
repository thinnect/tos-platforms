/**
 * @author Raido Pahtma
 * @license MIT
 */

module AvrWdtP {
	provides interface Init;
	uses interface Timer<TMilli>;
}
implementation {

	uint16_t m_wdt = 0;

	const char* labels[] = {
		"WDTO_15MS",  // 0
		"WDTO_30MS",  // 1
		"WDTO_60MS",  // 2
		"WDTO_120MS", // 3
		"WDTO_250MS", // 4
		"WDTO_500MS", // 5
		"WDTO_1S",    // 6
		"WDTO_2S",    // 7
		"WDTO_4S",    // 8
		"WDTO_8S",    // 9
	};

	uint32_t timeouts[] = {
		15,   // "WDTO_15MS",  // 0
		30,   // "WDTO_30MS",  // 1
		60,   // "WDTO_60MS",  // 2
		120,  // "WDTO_120MS", // 3
		250,  // "WDTO_250MS", // 4
		500,  // "WDTO_500MS", // 5
		1000, // "WDTO_1S",    // 6
		2000, // "WDTO_2S",    // 7
		4000, // "WDTO_4S",    // 8
		8000, // "WDTO_8S",    // 9
	};

	command error_t Init.init()
	{
		#ifdef TOSSIM_AVR_WDTON
			wdt_enable(WDTO_8S);
		#endif // TOSSIM_AVR_WDTON
		return SUCCESS;
	}

	void wdt_reset() @C()
	{
		dbg("avrwdt", "avrwdt: wdt_reset %s\n", labels[m_wdt]);
		call Timer.startOneShot(timeouts[m_wdt]);
	}

	void wdt_enable(uint16_t t) @C()
	{
		if(t <= 9)
		{
			m_wdt = t;
			dbg("avrwdt", "avrwdt: wdt_enable(%s)\n", labels[t]);
			wdt_reset();
		}
		else dbg("avrwdt", "avrwdt: wdt_enable value %u not supported!\n", t);
	}

	void wdt_disable() @C()
	{
		dbg("avrwdt", "avrwdt: wdt_disable()\n");
		call Timer.stop();
	}

	event void Timer.fired()
	{
		dbg("avrwdt", "avrwdt: wdt timeout exceeded\n");
	}

}
