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

		interface AMSend[uint8_t id];
		interface Receive[uint8_t id];
		interface Receive as Snoop[uint8_t id];
		interface SendNotifier[am_id_t id];

		interface Packet;
		interface AMPacket;

		interface PacketAcknowledgements;
		interface LowPowerListening;
#ifdef PACKET_LINK
		interface PacketLink;
#endif
		interface RadioChannel;

		interface PacketTimeStamp<TRadio, uint32_t> as PacketTimeStampRadio;
		interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;

		interface PacketField<uint8_t> as PacketLQI;
		interface PacketField<int8_t> as PacketRSSI;

#ifdef CONGESTION_CONTROL_ENABLED
		interface PacketField<bool> as PacketCongested;
		interface PacketField<bool> as PacketDropped;
#endif

		interface PacketField<uint8_t> as LinkType;

#ifdef LOW_POWER_LISTENING
		interface PacketField<uint16_t> as PacketRetryCount;
#endif
	}
}

implementation
{
	components RFA1ActiveMessageC as MessageC;

	SplitControl = MessageC;

	AMSend = MessageC;
	SendNotifier = MessageC;

	Packet = MessageC;
	AMPacket = MessageC;

	PacketAcknowledgements = MessageC;
	LowPowerListening = MessageC;
#ifdef PACKET_LINK
	PacketLink = MessageC;
#endif
	RadioChannel = MessageC;

	PacketTimeStampRadio = MessageC;
	PacketTimeStampMilli = MessageC;

	PacketLQI = MessageC.PacketLinkQuality;
	PacketRSSI = MessageC.PacketRSSI;

	LinkType = MessageC.LinkType;

#ifdef LOW_POWER_LISTENING
	PacketRetryCount = MessageC.PacketRetryCount;
#endif

#ifdef CONGESTION_CONTROL_ENABLED
	PacketCongested = MessageC.PacketCongested;
	PacketDropped = MessageC.PacketDropped;
#endif

	#ifdef AMFILTER
		#warning USING AMFILTER
		components AMFilterP, AMFILTER as Filter;
		Filter.AMPacket -> MessageC;

		AMFilterP.SubReceive -> MessageC.Receive;
		AMFilterP.SubSnoop -> MessageC.Snoop;
		AMFilterP.AMPacket -> MessageC;
		AMFilterP.AMFilter -> Filter;

		Receive = AMFilterP.Receive;
		Snoop = AMFilterP.Snoop;
	#else
		Receive = MessageC.Receive;
		Snoop = MessageC.Snoop;
	#endif
}
