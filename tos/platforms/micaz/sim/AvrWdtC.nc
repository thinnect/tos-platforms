/**
 * @author Raido Pahtma
 * @license MIT
 */
configuration AvrWdtC { }
implementation {

	components AvrWdtP;

	components MainC;
	MainC.SoftwareInit -> AvrWdtP.Init;

	components new TimerMilliC();
	AvrWdtP.Timer -> TimerMilliC;

}
