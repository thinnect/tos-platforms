/**
 * The most common PIR wiring for denode based motion detectors.
 *
 * @author Raido Pahtma
 * @license MIT
 */
configuration PlatformPirC {
	provides {
		interface Read<float>;
		interface Notify<float> as MovementStart;
		interface Notify<float> as MovementActive;
		interface Notify<float> as MovementEnd;
	}
}
implementation {

	#warning denode PIR

	components new RetriggeringPirC(FALSE, TRUE, 2000UL, 2000UL) as PIR;
	Read = PIR.Read;
	MovementStart = PIR.MovementStart;
	MovementActive = PIR.MovementActive;
	MovementEnd = PIR.MovementEnd;

	components HplAtm128GeneralIOC as GeneralIOC;
	PIR.InterruptPin -> GeneralIOC.PortB4;

	components new DummyGeneralIOC();
	PIR.PowerPin -> DummyGeneralIOC;

	components AtmegaPinChange0C as Interrupts;
	PIR.Interrupt -> Interrupts.GpioInterrupt[4];

}
