/**
 * @author Raido Pahtma
 * @license MIT
*/
configuration BatteryVoltageP {
	provides {
		interface Read<uint32_t> as VoltageRead[uint8_t client];
	}
}
implementation {

	components new InputVoltageReadP(uniqueCount("BatteryVoltageC"), 1600, 20, 10, 100);
	VoltageRead = InputVoltageReadP.VoltageRead;

	components HplAtm128GeneralIOC as Pins;
	InputVoltageReadP.ReadPin -> Pins.PortF0;
	InputVoltageReadP.EnablePin -> Pins.PortG2;

	components new DummyGeneralIOC();
	InputVoltageReadP.SinkPin -> DummyGeneralIOC;

	components new AdcReadClientC() as BatteryReadClientC;
	InputVoltageReadP.Read -> BatteryReadClientC;

	components new Atm128AdcConfigC(ATM128_ADC_SNGL_ADC0, ATM128_ADC_VREF_1_6, ATM128_ADC_PRESCALE);
	BatteryReadClientC.Atm128AdcConfig -> Atm128AdcConfigC;

}
