/*
 * Copyright (c) 2007, Vanderbilt University
 * All rights reserved.
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
 * - Neither the name of the copyright holder nor the names of
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
 *
 * Author: Miklos Maroti
 * Author: Andras Biro
 */
//#include <tos.h>
//#include <ActiveMessageLayer.h>
#include <RadioConfig.h>

#ifdef IEEE154FRAMES_ENABLED
#error "You cannot use ActiveMessageC with IEEE154FRAMES_ENABLED defined"
#endif

configuration SLActiveMessageC
{
	provides 
	{
		interface SplitControl;

		interface AMSend[am_id_t id];
		interface Receive[am_id_t id];
		interface Receive as Snoop[am_id_t id];
		interface SendNotifier[am_id_t id];
		interface Packet;
		interface AMPacket;

		interface PacketAcknowledgements;
		interface LowPowerListening;
		interface PacketLink;
		interface RadioChannel;

		interface PacketField<uint8_t> as PacketLinkQuality;
		interface PacketField<uint8_t> as PacketTransmitPower;
		interface PacketField<int8_t> as PacketRSSI;
		interface LinkPacketMetadata;

#if !defined(TFRAMES_ENABLED)  && !defined(IEEE154BARE_ENABLED)
		interface Ieee154Send;
		interface Receive as Ieee154Receive;
		interface SendNotifier as Ieee154Notifier;

		interface Resource as SendResource[uint8_t clint];

		interface Ieee154Packet;
		interface Packet as PacketForIeee154Message;
#endif

		interface LocalTime<TRadio> as LocalTimeRadio;
		interface PacketTimeStamp<TRadio, uint32_t> as PacketTimeStampRadio;
		interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;
	}
}

implementation
{
	components new NeighborhoodC(RFA1_NEIGHBORHOOD_SIZE);
	UniqueLayerC.Neighborhood -> NeighborhoodC;
	UniqueLayerC.NeighborhoodFlag -> NeighborhoodC.NeighborhoodFlag[unique("UQ_NEIGHBORHOOD_FLAG")];

	components new TaskletC();
	//components TimeSyncMessageC;
	components new ActiveMessageLayerC() as AMlayerC;
	components SLRadioP as RadioP;
	RadioP.PacketTimeStamp -> TimeStampingLayerC;
	RadioP.RadioAlarm -> RadioAlarmC.RadioAlarm[unique("UQ_RADIO_ALARM")];
	RadioP.SLPacket -> ActiveMessageLayerP;

	components LocalTimeMicroC;
	components new efm32CmsisTimerP(TRadio);
	components new AlarmRailMicroP(TRadio);
	components MainC;
	MainC.SoftwareInit -> efm32CmsisTimerP.Init;
	MainC.SoftwareInit -> AlarmRailMicroP.Init;
	//AMlayerC.SubPacket -> TinyosNetworkLayerC.TinyosPacket;

	AMlayerC.Config -> RadioP;
	AMlayerC.SubSend -> AutoResourceAcquireLayerC;
	AMlayerC.SubReceive -> TinyosNetworkLayerC.TinyosReceive;
	AMlayerC.SubPacket -> TinyosNetworkLayerC.TinyosPacket;

	TimeStampingLayerC.LocalTimeRadio -> LocalTimeMicroC;
	RadioAlarmC.Alarm -> efm32CmsisTimerP.Alarm;

	ActiveMessageLayerP.Alarm -> AlarmRailMicroP;

	AMSend = AMlayerC;
	Receive = AMlayerC.Receive;
	Snoop = AMlayerC.Snoop;
	SendNotifier = AMlayerC;
	Packet = AMlayerC;
	AMPacket = AMlayerC;
	LocalTimeRadio = LocalTimeMicroC;
	ActiveMessageLayerP.Counter -> AlarmRailMicroP;
	LinkPacketMetadata = ActiveMessageLayerP.LinkPacketMetadata;
	// TimeStampingLayerC.LocalTimeRadio -> ActiveMessageLayerP;

#if !defined(TFRAMES_ENABLED) && !defined(IEEE154BARE_ENABLED)
	components new SimpleFcfsArbiterC(RADIO_SEND_RESOURCE) as SendResourceC;
	SendResource = SendResourceC;

// -------- Ieee154 Message

	components new Ieee154MessageLayerC();
	Ieee154MessageLayerC.Ieee154PacketLayer -> Ieee154PacketLayerC;
	Ieee154MessageLayerC.SubSend -> TinyosNetworkLayerC.Ieee154Send;
	Ieee154MessageLayerC.SubReceive -> TinyosNetworkLayerC.Ieee154Receive;
	Ieee154MessageLayerC.RadioPacket -> TinyosNetworkLayerC.Ieee154Packet;

	Ieee154Send = Ieee154MessageLayerC;
	Ieee154Receive = Ieee154MessageLayerC;
	Ieee154Notifier = Ieee154MessageLayerC;
	Ieee154Packet = Ieee154PacketLayerC;
	PacketForIeee154Message = Ieee154MessageLayerC;
#endif

	components new MetadataFlagsLayerC();
	TimeStampingLayerC.TimeStampFlag -> MetadataFlagsLayerC.PacketFlag[unique("UQ_METADATA_FLAGS")];
	MetadataFlagsLayerC.SubPacket -> ActiveMessageLayerP;

	components new RadioAlarmC();
	SoftwareAckLayerC.RadioAlarm -> RadioAlarmC.RadioAlarm[unique("UQ_SL_RADIO_ALARM")];
	RadioAlarmC.Tasklet -> TaskletC;
	components new TinyosNetworkLayerC();

#ifndef IEEE154BARE_ENABLED
	

	TinyosNetworkLayerC.SubSend -> UniqueLayerC;
	TinyosNetworkLayerC.SubReceive -> Ieee154PacketLayerC;
	TinyosNetworkLayerC.SubPacket -> Ieee154PacketLayerC;
#endif

	UniqueLayerC.SubReceive -> CollisionAvoidanceLayerC;

	components new RandomCollisionLayerC() as CollisionAvoidanceLayerC;
	CollisionAvoidanceLayerC.Config -> RadioP.RandomCollisionConfig;
	CollisionAvoidanceLayerC.SubSend -> SoftwareAckLayerC;
	CollisionAvoidanceLayerC.SubReceive -> SoftwareAckLayerC;
	CollisionAvoidanceLayerC.RadioAlarm -> RadioAlarmC.RadioAlarm[unique("UQ_RADIO_ALARM")];

	components new MessageBufferLayerC();
	MessageBufferLayerC.RadioSend -> CollisionAvoidanceLayerC;
	MessageBufferLayerC.RadioState -> TrafficMonitorLayerC;
	// MessageBufferLayerC.RadioState -> ActiveMessageLayerP;
	MessageBufferLayerC.Tasklet -> TaskletC;
	MessageBufferLayerC.RadioReceive -> UniqueLayerC;
	RadioChannel = MessageBufferLayerC;
	// RadioChannel = ActiveMessageLayerP;

	components new LowPowerListeningDummyC() as LowPowerListeningLayerC;
	LowPowerListeningLayerC.SubControl -> MessageBufferLayerC;
	LowPowerListeningLayerC.SubSend -> MessageBufferLayerC;
	LowPowerListeningLayerC.SubReceive -> MessageBufferLayerC;
	LowPowerListeningLayerC.SubPacket -> TimeStampingLayerC;
	// SplitControl -> ActiveMessageLayerP.RadioState;
	SplitControl = LowPowerListeningLayerC;
	LowPowerListening = LowPowerListeningLayerC;

	components new TrafficMonitorLayerC();
	TrafficMonitorLayerC.Config -> RadioP;
	TrafficMonitorLayerC -> RadioDriverDebugLayerC.RadioSend;
	// TrafficMonitorLayerC -> ActiveMessageLayerP.RadioState;
	TrafficMonitorLayerC -> RadioDriverDebugLayerC.RadioState;

	components new DummyLayerC() as CsmaLayerC;
	CsmaLayerC.Config -> RadioP;
	CsmaLayerC -> TrafficMonitorLayerC.RadioSend;
	CsmaLayerC -> TrafficMonitorLayerC.RadioReceive;

	components new DummyLayerC() as RadioDriverDebugLayerC;
	RadioDriverDebugLayerC.SubState -> ActiveMessageLayerP;

	components new SoftwareAckLayerC();
	ActiveMessageLayerP.AckReceivedFlag -> MetadataFlagsLayerC.PacketFlag[unique("UQ_METADATA_FLAGS")];
	SoftwareAckLayerC.Config -> RadioP.SoftwareAckConfig;
	SoftwareAckLayerC.SubSend -> CsmaLayerC;
	SoftwareAckLayerC.SubReceive -> CsmaLayerC;
	PacketAcknowledgements = ActiveMessageLayerP;

	components new PacketLinkLayerC();
	PacketLink = PacketLinkLayerC;
	PacketLinkLayerC.PacketAcknowledgements -> ActiveMessageLayerP;
	PacketLinkLayerC -> LowPowerListeningLayerC.Send;
	PacketLinkLayerC -> LowPowerListeningLayerC.Receive;
	PacketLinkLayerC -> LowPowerListeningLayerC.RadioPacket;

	components new TimeStampingLayerC();
	// LowPowerListeningLayerC.SubPacket -> TimeStampingLayerC;
	PacketTimeStampRadio = TimeStampingLayerC;
	PacketTimeStampMilli = TimeStampingLayerC;
	TimeStampingLayerC.SubPacket -> MetadataFlagsLayerC;
	
	components new Ieee154PacketLayerC();
	Ieee154PacketLayerC.SubPacket -> PacketLinkLayerC;
#ifndef IEEE154BARE_ENABLED
	//some layers needs this to understand the ieee154 header,
	//but we don't want to actually process it in IEEE154BARE mode
	Ieee154PacketLayerC.SubReceive -> PacketLinkLayerC;
#endif
	
	components ActiveMessageLayerP;
	PacketRSSI = ActiveMessageLayerP.PacketRSSI;
	PacketTransmitPower = ActiveMessageLayerP.PacketTransmitPower;
	PacketLinkQuality = ActiveMessageLayerP.PacketLinkQuality;
	ActiveMessageLayerP.PacketTimeStamp -> TimeStampingLayerC;
	ActiveMessageLayerP.Ieee154Packet -> Ieee154PacketLayerC;
	RadioP.Ieee154PacketLayer -> Ieee154PacketLayerC;

#if !defined(TFRAMES_ENABLED)
	//components new Ieee154MessageLayerC();
	Ieee154MessageLayerC.SubSend -> TinyosNetworkLayerC.Ieee154Send;
	Ieee154MessageLayerC.SubReceive -> TinyosNetworkLayerC.Ieee154Receive;
	Ieee154MessageLayerC.RadioPacket -> TinyosNetworkLayerC.Ieee154Packet;
#endif

#ifdef IEEE154BARE_ENABLED
	components new BlipCompatibilityLayerC();
	BlipCompatibilityLayerC.SubSend -> UniqueLayerC;
	BlipCompatibilityLayerC.SubReceive -> PacketLinkLayerC;
	BlipCompatibilityLayerC.SubPacket -> PacketLinkLayerC;
	BlipCompatibilityLayerC.SubLqi -> ActiveMessageLayerC.PacketLinkQuality;
	BlipCompatibilityLayerC.SubRssi -> ActiveMessageLayerC.PacketRSSI;
	
	Send = BlipCompatibilityLayerC;
	Receive = BlipCompatibilityLayerC;
	Packet = BlipCompatibilityLayerC;
	Ieee154Address = BlipCompatibilityLayerC;
	ReadLqi = BlipCompatibilityLayerC;
#endif

	components new UniqueLayerC(RFA1_NEIGHBORHOOD_SIZE);
	UniqueLayerC.Config -> RadioP.UniqueConfig;
	UniqueLayerC.SubSend -> PacketLinkLayerC;

	components new DummyLayerC() as AutoResourceAcquireLayerC;
	AutoResourceAcquireLayerC -> TinyosNetworkLayerC.TinyosSend;

	ActiveMessageLayerP.TransmitPowerFlag -> MetadataFlagsLayerC.PacketFlag[unique("UQ_METADATA_FLAGS")];
	ActiveMessageLayerP.RSSIFlag -> MetadataFlagsLayerC.PacketFlag[unique("UQ_METADATA_FLAGS")];
	ActiveMessageLayerP.TimeSyncFlag -> MetadataFlagsLayerC.PacketFlag[unique("UQ_METADATA_FLAGS")];
}
