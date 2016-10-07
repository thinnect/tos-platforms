/**
 * Denode platform wiring for DS2401 onewire pin.
 * @author Raido Pahtma
 * @license MIT
 */
configuration Ds2401OneWirePinC {
	provides interface GeneralIO as Pin;
}
implementation {

	components HplAtm128GeneralIOC;
	Pin = HplAtm128GeneralIOC.PortE4;

}
