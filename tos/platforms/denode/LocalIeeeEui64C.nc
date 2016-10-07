/**
 * Denode platform LocalIeeeEui64C from DS2401 chip. Not a proper EUI64.
 * @author Raido Pahtma
 * @license MIT
 */
configuration LocalIeeeEui64C {
	provides interface LocalIeeeEui64;
}
implementation {

    components Ds2401Eui64C;
    LocalIeeeEui64 = Ds2401Eui64C.LocalIeeeEui64;

}
