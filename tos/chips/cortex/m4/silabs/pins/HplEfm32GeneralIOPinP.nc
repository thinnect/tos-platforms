/// $Id: HplAtm128GeneralIOPinP.nc,v 1.8 2010-06-29 22:07:43 scipio Exp $

/*
 * Copyright (c) 2004-2005 Crossbow Technology, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of Crossbow Technology nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Generic pin access for pins mapped into I/O space (for which the sbi, cbi
 * instructions give atomic updates). This can be used for ports A-E.
 *
 * @author Martin Turon <mturon@xbow.com>
 * @author David Gay <dgay@intel-research.net>
 */
generic module HplEfm32GeneralIOPinP (uint32_t port, uint32_t pin)
{
  provides interface GeneralIO as IO;
}
implementation
{
//#define pin (*TCAST(volatile uint32_t * ONE, pin))

//  IF THIS DOESN't WORK -> point to struct type through LPC_GPIOx -> STRUCT DEFINITIONS
// portf f mask 0xFFFF
  async command bool IO.get() {
    // return (GPIO->P[port].DOUT) ? 1 : 0;
  }

  async command void IO.set() {
    // BUS_RegBitWrite(&GPIO->P[port].DOUT, pin, 1);
  }

  async command void IO.clr() {
    // BUS_RegBitWrite(&GPIO->P[port].DOUT, pin, 0);
  }

  async command void IO.toggle() { 
    // GPIO->P[port].DOUTTGL = 1 << pin;
  }
    
  async command void IO.makeInput() {
    // GPIO->P[port].MODEL = (GPIO->P[port].MODEL & ~(0xFu << (pin * 4)))
    //                       | (_GPIO_P_MODEL_MODE0_INPUT << (pin * 4));
  }

  async command void IO.makeOutput() {
    // GPIO->P[port].MODEL = (GPIO->P[port].MODEL & ~(0xFu << (pin * 4)))
    //                       | (_GPIO_P_MODEL_MODE0_PUSHPULL << (pin * 4));

  }

  async command bool IO.isInput() {

  }

  async command bool IO.isOutput() {

  }
}
