/**
 * Platform-specific LED interface.
 *
 * @author Martin Turon <mturon@xbow.com>
 */

#include "hardware.h"

generic module HilEfm32GpioC
{
  provides interface GeneralIO;
  provides interface GpioInterrupt;
  uses interface HplEfm32GeneralIO as Efm32GeneralIO;
  uses interface HplEfm32GpioInterrupt as Efm32GpioInterrupt;
  uses interface Init;
}
implementation
{
  .
  .
  .
}
