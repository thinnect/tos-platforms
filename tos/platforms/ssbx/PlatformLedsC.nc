/* SuperSoapBoX leds */
configuration PlatformLedsC {
  provides {
    interface GeneralIO as Led0;
    interface GeneralIO as Led1;
    interface GeneralIO as Led2;
  }
  uses interface Init;
}
implementation {

  components HplAtm128GeneralIOC as IO;

  components PlatformC;
  Init = PlatformC.LedsInit;
  Led0 = IO.PortE6;
  Led1 = IO.PortE5;
  Led2 = IO.PortD7;

}
