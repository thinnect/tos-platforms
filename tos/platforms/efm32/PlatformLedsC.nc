/**
 * Platform-specific LED interface.
 *
 * @author Martin Turon <mturon@xbow.com>
 */

#include "hardware.h"

configuration PlatformLedsC
{
  provides interface GeneralIO as Led0; //HplEfm32
  provides interface GeneralIO as Led1;
  provides interface GeneralIO as Led2;
  uses interface Init;
}
implementation
{
  components HplEfm32GeneralIOC as GeneralIO;
  components PlatformP;

  Init = PlatformP.LedsInit;

  Led0 = GeneralIO.Led0;
  Led1 = GeneralIO.Led1;
  Led2 = GeneralIO.Led2;  
}
