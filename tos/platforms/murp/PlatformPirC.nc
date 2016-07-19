/**
 * The most common PIR wiring for MURP module based motion detectors.
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

	components new RetriggeringPirC(FALSE, TRUE, 5000UL, 5000UL) as PIR;
	Read = PIR.Read;
	MovementStart = PIR.MovementStart;
	MovementActive = PIR.MovementActive;
	MovementEnd = PIR.MovementEnd;

	components HplAtm128GeneralIOC as GeneralIOC;
	PIR.InterruptPin -> GeneralIOC.PortD3;

	components AtmegaExtInterruptC as Interrupts;
	PIR.Interrupt -> Interrupts.GpioInterrupt[3];

}
