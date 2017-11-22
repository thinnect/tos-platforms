/**
 * HPL for the mbed LPC 1768 microprocessor. This provides an
 * abstraction for general-purpose I/O.
 *
 */

#include "efr32mg12p332f1024gl125.h"
#include "em_bus.h"

configuration HplEfm32GeneralIOC
{
  // provides all the ports as raw ports
  provides {
    interface GeneralIO as Led0;
    interface GeneralIO as Led1; //HplEfm32
    interface GeneralIO as Led2;
  }
}

implementation
{
  components 

    new HplEfm32GeneralIOPinP(5, 4) as LED0, // PF4
    new HplEfm32GeneralIOPinP(5, 5) as LED1, // PF5
    new HplEfm32GeneralIOPinP(5, 5) as LED2; //  P1_20

    //PlatformC; // dummy to end unknown sequence
  Led0 = LED0;
  Led1 = LED1;
  Led2 = LED2;
}
