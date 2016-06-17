/**
 * The most common PIR wiring for denode based motion detectors.
 *
 * @author Raido Pahtma
 * @license Thinnect
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

	components new RetriggeringPirC(FALSE, TRUE, 2000UL, 2000UL) as PIR;
	Read = PIR.Read;
	MovementStart = PIR.MovementStart;
	MovementActive = PIR.MovementActive;
	MovementEnd = PIR.MovementEnd;

	components HplAtm128GeneralIOC as GeneralIOC;
	PIR.InterruptPin -> GeneralIOC.PortB4;

    components AtmegaPinChange0C;
    PIR.Interrupt -> AtmegaPinChange0C.GpioInterrupt[4];

}
