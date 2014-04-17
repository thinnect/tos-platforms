configuration AvrWdtC {

}
implementation {

	components AvrWdtP;

	components new TimerMilliC();
	AvrWdtP.Timer -> TimerMilliC;

}
