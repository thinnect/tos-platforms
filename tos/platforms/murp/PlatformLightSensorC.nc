/**
 * Platform LightSensor for murpdome. Requires VEML7700 driver.
 *
 * @author Raido Pahtma
 * @license MIT
 */
configuration PlatformLightSensorC {
	provides interface Read<uint32_t>;
}
implementation {

	components new VEML7700C(50, 1000) as LightSensor;
	Read = LightSensor;

	components new Atm128I2CMasterC();
	LightSensor.Resource -> Atm128I2CMasterC;
	LightSensor.I2C -> Atm128I2CMasterC;

	components AtmegaGeneralIOC;
	LightSensor.VDD -> AtmegaGeneralIOC.PortE5;

}
