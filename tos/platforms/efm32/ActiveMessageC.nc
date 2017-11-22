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
 * Author: Miklos Maroti
 */

#include <RadioConfig.h>

configuration ActiveMessageC
{
	provides
	{
		interface SplitControl;

		interface AMSend[am_id_t id];
		interface Receive[uint8_t id];
		interface Receive as Snoop[uint8_t id];
		interface SendNotifier[am_id_t id];

		interface Packet;
		interface AMPacket;

		interface PacketAcknowledgements;
		interface LowPowerListening; // leave empty
		interface PacketLink; // leave empty
		interface RadioChannel; // leave empty

		interface PacketField<uint8_t> as PacketLinkQuality;
		interface PacketField<int8_t> as PacketRSSI;

		interface LocalTime<TRadio> as LocalTimeRadio;
		interface PacketTimeStamp<TRadio, uint32_t> as PacketTimeStampRadio;
		interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;
	}
}

implementation
{
	components SLActiveMessageC as MessageC;

	SplitControl = MessageC;

	AMSend = MessageC.AMSend;
	Receive = MessageC.Receive;
	Snoop = MessageC.Snoop;
	SendNotifier = MessageC;

	Packet = MessageC.Packet;
	AMPacket = MessageC;

	LocalTimeRadio = MessageC;

	PacketAcknowledgements = MessageC;
	LowPowerListening = MessageC;
	PacketLink = MessageC;
	RadioChannel = MessageC;

	PacketLinkQuality = MessageC.PacketLinkQuality;
	PacketRSSI = MessageC.PacketRSSI;

	PacketTimeStampRadio = MessageC;
	PacketTimeStampMilli = MessageC;
}
