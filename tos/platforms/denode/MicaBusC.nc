configuration MicaBusC {
	provides {
		interface MicaBusAdc as Adc0;
		interface MicaBusAdc as Adc1;
		interface MicaBusAdc as Adc2;
		interface MicaBusAdc as Adc3;
		interface MicaBusAdc as Adc4;
		interface MicaBusAdc as Adc5;
		interface MicaBusAdc as Adc6;
		interface MicaBusAdc as Adc7;
	}
}
implementation {

	components MicaBusP;
	Adc0 = MicaBusP.Adc0;
	Adc1 = MicaBusP.Adc1;
	Adc2 = MicaBusP.Adc2;
	Adc3 = MicaBusP.Adc3;
	Adc4 = MicaBusP.Adc4;
	Adc5 = MicaBusP.Adc5;
	Adc6 = MicaBusP.Adc6;
	Adc7 = MicaBusP.Adc7;

}
