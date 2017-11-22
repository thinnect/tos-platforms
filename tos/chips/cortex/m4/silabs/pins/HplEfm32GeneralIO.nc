/* HIL GeneralIO interface for LPC1768*/

#include "TinyError.h"

interface HplEfm32GeneralIO
{
  /*** Set pin to high.*/
  async command void set();

  /*** Set pin to low.*/
  async command void clr();

  /*** Toggle pin status.*/
  async command void toggle();

  /**
   * Get the port status that contains the pin.
   *
   * @return Status of the port that contains the given pin. The x'th
   * pin on the port will be represented in the x'th bit.
   */

  /*** Read pin value.** @return TRUE if pin is high, FALSE otherwise.*/
  async command bool get();

  /*** Set pin direction to input.*/
  async command void makeInput();
  
  /*** Set pin direction to output.*/
  async command void makeOutput();

  async command bool isOutput();

  async command bool isInput();
}

