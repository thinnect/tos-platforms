/*
 * Copyright (c) 2007, Vanderbilt University
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE VANDERBILT UNIVERSITY BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE VANDERBILT
 * UNIVERSITY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE VANDERBILT UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE VANDERBILT UNIVERSITY HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 */

#ifndef PLATFORM_MESSAGE_H
#define PLATFORM_MESSAGE_H

#include <RFA1Radio.h>
#include "ChipconPacket.h"
#include <Serial.h>
#include "LiteSerial.h"

typedef union message_header {
	rfa1packet_header_t rfa1;
	chipcon_header_t cc1101;
	serial_header_t serial;
	lite_serial_header_r lite_serial;
} message_header_t;

typedef union message_footer {
	rfa1packet_footer_t rfa1;
	chipcon_footer_t cc1101;
} message_footer_t;

typedef union message_metadata {
	rfa1packet_metadata_t rfa1;
	chipcon_metadata_t cc1101;
	lite_serial_metadata_t lite_serial;
} message_metadata_t;

#endif
