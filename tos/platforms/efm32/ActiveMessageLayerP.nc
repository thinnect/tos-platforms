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
 * Author: Chieh-Jan Mike Liang (default interfaces for TOSThreads)
 */


//#include <tos.h>
#include <Tasklet.h>
#include <SLDriverLayer.h>
#include <Ieee154PacketLayer.h>
#include <ActiveMessageLayer.h>
#include <MetadataFlagsLayer.h>
#include <RadioConfig.h>
#include <TimeSyncMessageLayer.h>
#include "rail.h"
#include "rail_chip_specific.h"
#include "rail_types.h"
#include "main.h"
#include "em_msc.h"

// generic module ActiveMessageLayerP()
module ActiveMessageLayerP
{
	provides
	{
		interface RadioPacket;
		interface AMPacket;
		interface Packet;
		interface AMSend[am_id_t id];
		interface Receive[am_id_t id];
		interface Receive as Snoop[am_id_t id];	
		interface SendNotifier[am_id_t id];
		interface PacketField<uint8_t> as PacketTimeSyncOffset;
		interface PacketField<uint8_t> as PacketTransmitPower;
		interface PacketField<uint8_t> as PacketLinkQuality;
		interface PacketField<int8_t> as PacketRSSI;
		interface LinkPacketMetadata;
		interface RadioState;
		interface PacketAcknowledgements;
	}

	uses
	{
		interface PacketFlag as TimeSyncFlag;
		interface PacketFlag as TransmitPowerFlag;
		interface PacketFlag as RSSIFlag;
		interface PacketFlag as AckReceivedFlag;
		interface Counter<TRadio, uint32_t>;
		interface Alarm<TRadio, uint32_t>;
		interface UniqueConfig;
		interface Ieee154Packet;
		interface RadioPacket as SubPacket;
		interface BareSend as SubSend;
		interface BareReceive as SubReceive;
		interface ActiveMessageConfig as Config;
		interface ActiveMessageAddress;
		interface Leds;
		interface PacketTimeStamp<TRadio, uint32_t>;
	}
}

implementation
{
	task void rcvdTask();
	task void sntTask();

	#define __MODUUL__ "activeMessage"
	#define __LOG_LEVEL__ ( LOG_LEVEL_ActiveMessageP & BASE_LOG_LEVEL )
	#include "log.h"

	#define CHANNEL_ADDR	(0x0FE00004)
	#define TOS_ID_ADDR		(0x0FE00000)

	message_t *msgCopy;
	uint8_t sequenceNumber = 0;
	message_t g_msg;
	message_t *g_msg_copy = &g_msg;
	uint8_t busy = 0;
	uint8_t sntStatus = 0;
	uint8_t needAck = 0;
	uint8_t channel = 0;

	uint8_t txData[114] = {0};

	task void sntTask() 
	{
		if (msgCopy != NULL) {
			if (sntStatus == 1) {
				signal SubSend.sendDone(msgCopy, SUCCESS);
			} else if (sntStatus == 2){
				err1("EBUSY SNT DONE");
				signal SubSend.sendDone(msgCopy, EBUSY);
			} else if (sntStatus == 0) {
				err1("FAIL SNT DONE");
				signal SubSend.sendDone(msgCopy, FAIL);
			}
			msgCopy = NULL;
			sntStatus = 0;
		} else {
			err1("msgCopy is NULL, sntStatus=%u", sntStatus);
		}
	}

	void packetSent(uint8_t status) @C() @spontaneous()
	{	
		sntStatus = status;

		if (needAck == 1 && sntStatus == 1) {
			needAck = 0;
			pinSet();
			call Alarm.start(RAIL_GetTime() + 892);
		} else {
			post sntTask();
		}
	}

	void rxAckTimeout(void) @C() @spontaneous()
	{
		call Alarm.stop();
		err1("rxAckTimeout");
		sntStatus = 0;
		call AckReceivedFlag.setValue(msgCopy, false);
		post sntTask();
	}

	async event void Alarm.fired() {
		pinClear();
		// warn1("alarm fired");
		call AckReceivedFlag.setValue(msgCopy, true);
		post sntTask();
	}

	async event void Counter.overflow() {
		err1("Counter overflow");
	}

	slpacket_header_t* getHeader(message_t* msg) // sl_header_t
	{
		return ((void*)msg) + offsetof(message_t, data) - sizeof(message_header_t);
	}

	activemessage_header_t* getHeaderAM(message_t* msg)
	{
		return ((void*)msg) + offsetof(message_t, data) - sizeof(sl_header_t);
	}

	ieee154_simple_header_t* getIeeeHeader(message_t* msg)
	{
		return (ieee154_simple_header_t*) (void*)msg;
	}

	void* getPayload(message_t* msg)
	{
		return ((void*)msg) + call RadioPacket.headerLength(msg);
	}

	void setSequenceNumber(message_t* msg, uint8_t dsn)
	{
		getIeeeHeader(msg)->dsn = dsn;
	}

	task void rcvdTask()
	{
		g_msg_copy = signal SubReceive.receive(g_msg_copy);

		busy = 0;
	}

	void packetReceived(RAIL_Handle_t rxHandle) @C() @spontaneous()
	{
		uint8_t i = 0;
		uint8_t pos = 0;
		uint8_t lastPortCnt = 0;
		uint32_t rxTime = 0;
		RAIL_RxPacketDetails_t appendedInfo;
		RAIL_RxPacketInfo_t rxPacketInfo;
		RAIL_RxPacketHandle_t packetHandle;

		if (busy == 0) {

			busy = 1;
			packetHandle = RAIL_GetRxPacketInfo(rxHandle, RAIL_RX_PACKET_HANDLE_NEWEST,
			                           &rxPacketInfo);
			RAIL_GetRxPacketDetails(rxHandle, packetHandle, &appendedInfo);

			pin2set();

			if (rxPacketInfo.firstPortionBytes == rxPacketInfo.packetBytes) {
				for (i = 0; i < sizeof(message_header_t); i++) {
					g_msg_copy->header[i] = rxPacketInfo.firstPortionData[i+1];
				}

				for (i = 0; i < (rxPacketInfo.packetBytes - sizeof(message_header_t) - 1); i++) {
					g_msg_copy->data[i] = rxPacketInfo.firstPortionData[12+i]; // 12 - payload start position, define!
				}
			} else {
				if (rxPacketInfo.firstPortionBytes < (sizeof(message_header_t) + 1)) {
					for (i = 0; i < rxPacketInfo.firstPortionBytes-1; i++) {
						g_msg_copy->header[i] = rxPacketInfo.firstPortionData[i+1];
					}
					for (pos = i; pos < sizeof(message_header_t); pos++) {
						g_msg_copy->header[pos] = rxPacketInfo.lastPortionData[lastPortCnt];
						lastPortCnt++;	
					}
					for (i = 0; i < (rxPacketInfo.packetBytes - sizeof(message_header_t) - 1); i++) {
						g_msg_copy->data[i] = rxPacketInfo.lastPortionData[lastPortCnt+i];
					}
				} else if (rxPacketInfo.firstPortionBytes > (sizeof(message_header_t) + 1)) {
					for (i = 0; i < sizeof(message_header_t); i++) {
						g_msg_copy->header[i] = rxPacketInfo.firstPortionData[i+1];
					}
					for (i = 0; i < (rxPacketInfo.firstPortionBytes - sizeof(message_header_t)); i++) {
						g_msg_copy->data[i] = rxPacketInfo.firstPortionData[12+i];
					}
					for (lastPortCnt = 0; lastPortCnt < (rxPacketInfo.packetBytes - rxPacketInfo.firstPortionBytes); lastPortCnt++) {
						g_msg_copy->data[i-1] = rxPacketInfo.lastPortionData[lastPortCnt];
						i++;
					}
				} else {
					for (i = 0; i < sizeof(message_header_t); i++) {
						g_msg_copy->header[i] = rxPacketInfo.firstPortionData[i+1];
					}
					for (i = 0; i < (rxPacketInfo.packetBytes - rxPacketInfo.firstPortionBytes); i++) {
						g_msg_copy->data[i] = rxPacketInfo.lastPortionData[i];
					}
				}
			}
			getHeader(g_msg_copy)->length = rxPacketInfo.packetBytes - 12;

			call PacketTimeStamp.set(g_msg_copy, call Counter.get() / 16);
			call PacketRSSI.set(g_msg_copy, appendedInfo.rssi);
			call PacketLinkQuality.set(g_msg_copy, appendedInfo.lqi);
			
			post rcvdTask();
		}
	}

/*----------------- Send -----------------*/

	command error_t AMSend.send[am_id_t id](am_addr_t addr, message_t* msg, uint8_t len)
	{
		uint8_t sl_message[255] = {0};
		uint8_t dataCnt = 0;
		uint8_t i = 0;
		uint32_t time;
		uint8_t offset;
		uint32_t msgTimestamp;
		int32_t txTimestamp;
		uint8_t n = 0;
		uint16_t dataLen = 0;
		uint8_t dataFifo = 0;
		RAIL_TxOptions_t txOption;

		RAIL_CsmaConfig_t csmaConf = {3, 5, 5, -75, 320, 128, 0};

		osThreadId_t thid;
		osStatus_t status;

		if( len > call Packet.maxPayloadLength() )
			return EINVAL;

		call Packet.setPayloadLength(msg, len); //+13
 		call AMPacket.setSource(msg, call AMPacket.address());
		call AMPacket.setGroup(msg, call AMPacket.localGroup());
		call AMPacket.setType(msg, id);
		call AMPacket.setDestination(msg, addr);
		setSequenceNumber(msg, ++sequenceNumber);

		if (railHandle == NULL) {
			err1("Could not initialize RAIL");
		}

		dataLen = len + sizeof(message_header_t) + 1;
		txData[0] = len + sizeof(message_header_t) + 2;

		msg->header[9] = 0x3f;
		for (i = 0; i < sizeof(message_header_t); i++) {
			txData[i+1] = msg->header[i];
		}

		if (addr == 0xffff) {
			needAck = 0;
			txData[1] = 0x41;
		} else {
			needAck = 1;
			txData[1] = 0x61;
		}

		txData[2] = 0x88;

		if (id == 0x3D) {
			if (call PacketTimeSyncOffset.isSet(msg)) {

				offset = call PacketTimeSyncOffset.get(msg);
				offset = offset - sizeof(message_header_t);
				msgTimestamp = (msg->data[offset] << 24) | (msg->data[offset+1] << 16) | 
								(msg->data[offset+2] << 8) | (msg->data[offset+3]);
				
				txTimestamp = msgTimestamp - call Counter.get() / 16;
				// debug1("msgTimestamp=%u", msgTimestamp);
				// debug1("call Counter.get()=%u", call Counter.get() / 16);
				msg->data[offset] = txTimestamp >> 24;
				msg->data[offset+1] = txTimestamp >> 16;
				msg->data[offset+2] = txTimestamp >> 8;
				msg->data[offset+3] = txTimestamp;

				call PacketTimeStamp.set(msg, call Counter.get() / 16);
			}
		}

		for (dataCnt = 0; dataCnt < len; dataCnt++) {
			txData[i+1] = msg->data[dataCnt];
			i++;
		}

		dataFifo = RAIL_WriteTxFifo(railHandle, txData, dataLen, true);
		msgCopy = msg;
		signal SendNotifier.aboutToSend[id](addr, msg); //msgCopy

		if (RAIL_StartCcaCsmaTx(railHandle, channel, 0, &csmaConf, NULL)) { //&RAIL_CcaCsma, &csmaConf
			warn1("snt fail");
			return FAIL;
		}

		return SUCCESS;
	}

	inline event void SubSend.sendDone(message_t* msg, error_t error)
	{
		signal AMSend.sendDone[call AMPacket.type(msg)](msg, error);
	}

	inline command error_t AMSend.cancel[am_id_t id](message_t* msg)
	{
		return call SubSend.cancel(msg);
	}

	default event void AMSend.sendDone[am_id_t id](message_t* msg, error_t error)
	{
	}

	inline command uint8_t AMSend.maxPayloadLength[am_id_t id]()
	{
		return call Packet.maxPayloadLength();
	}

	inline command void* AMSend.getPayload[am_id_t id](message_t* msg, uint8_t len)
	{
		return call Packet.getPayload(msg, len);
	}

	default event void SendNotifier.aboutToSend[am_id_t id](am_addr_t addr, message_t* msg)
	{
	}

/*----------------- Receive -----------------*/

	event message_t* SubReceive.receive(message_t* msg)
	{			
		uint8_t i = 0;
		am_id_t id = call AMPacket.type(msg);

		void* payload = getPayload(msg);
		uint8_t len = call Packet.payloadLength(msg);



		if (call AMPacket.isForMe(msg)) {
			msg = signal Receive.receive[id](msg, payload, len);
		} else {
			msg = signal Snoop.receive[id](msg, payload, len);
		}

		pin2clear();
		return msg;
	}

	default event message_t* Receive.receive[am_id_t id](message_t* msg, void* payload, uint8_t len)
	{
                return msg;
	}

	default event message_t* Snoop.receive[am_id_t id](message_t* msg, void* payload, uint8_t len)
	{
                return msg;
	}

/*----------------- AMPacket -----------------*/

	inline command am_addr_t AMPacket.address()
	{
		return call ActiveMessageAddress.amAddress();
	}
 
	inline command am_group_t AMPacket.localGroup()
	{
		return call ActiveMessageAddress.amGroup();
	}

	inline command uint8_t AMPacket.isForMe(message_t* msg)
	{
		am_addr_t addr = call AMPacket.destination(msg);

		return (addr == AM_BROADCAST_ADDR || addr == call AMPacket.address()) && call AMPacket.group(msg) == call AMPacket.localGroup();
	}

	inline command am_addr_t AMPacket.destination(message_t* msg)
	{
		return getIeeeHeader(msg)->dest;
	}
 
	inline command void AMPacket.setDestination(message_t* msg, am_addr_t addr)
	{
		getIeeeHeader(msg)->dest = addr;
	}

	inline command am_addr_t AMPacket.source(message_t* msg)
	{
		return getIeeeHeader(msg)->src;
	}

	inline command void AMPacket.setSource(message_t* msg, am_addr_t addr)
	{
		getIeeeHeader(msg)->src = addr;
	}

	inline command am_id_t AMPacket.type(message_t* msg)
	{
		return getHeaderAM(msg)->type;
	}

	inline command void AMPacket.setType(message_t* msg, am_id_t type)
	{
		getHeaderAM(msg)->type = type;
	}
  
	inline command am_group_t AMPacket.group(message_t* msg) 
	{
		return getIeeeHeader(msg)->destpan;
	}

	inline command void AMPacket.setGroup(message_t* msg, am_group_t grp)
	{
		call Ieee154Packet.setPan(msg, grp);
	}

	inline async event void ActiveMessageAddress.changed()
	{
	}

/*----------------- RadioPacket -----------------*/

	async command uint8_t RadioPacket.headerLength(message_t* msg)
	{
		return sizeof(message_header_t);
	}

	async command uint8_t RadioPacket.payloadLength(message_t* msg)
	{
		return getHeader(msg)->length;
	}

	async command void RadioPacket.setPayloadLength(message_t* msg, uint8_t length)
	{
		getHeader(msg)->length = length;
	}

	async command uint8_t RadioPacket.maxPayloadLength()
	{
		return sizeof(slpacket_header_t) + TOSH_DATA_LENGTH - sizeof(activemessage_header_t);
	}

	async command uint8_t RadioPacket.metadataLength(message_t* msg)
	{
		return sizeof(message_metadata_t);
	}



	async command void RadioPacket.clear(message_t* msg)
	{
		// debug1("RadioPacket.clear");
		// call SubPacket.clear(msg);
	}

/*----------------- Packet -----------------*/

	command void Packet.clear(message_t* msg)
	{
		call RadioPacket.clear(msg);
	}

	command uint8_t Packet.payloadLength(message_t* msg)
	{
		return call RadioPacket.payloadLength(msg);
	}

	command void Packet.setPayloadLength(message_t* msg, uint8_t len)
	{
		call RadioPacket.setPayloadLength(msg, len);
	}

	command uint8_t Packet.maxPayloadLength()
	{
		return call RadioPacket.maxPayloadLength();
	}

	command void* Packet.getPayload(message_t* msg, uint8_t len)
	{
		if( len > call RadioPacket.maxPayloadLength() )
			return NULL;

		return ((void*)msg) + call RadioPacket.headerLength(msg);
	}

	flags_metadata_t* getMetadata(message_t* msg)
	{
		return ((void*)msg) + sizeof(message_t) - call RadioPacket.metadataLength(msg);
	}

	sl_metadata_t* getMeta(message_t* msg)
	{
		return ((void*)msg) + sizeof(message_t) - call RadioPacket.metadataLength(msg);
	}

	/*----------------- PacketTimeSyncOffset -----------------*/

	async command uint8_t PacketTimeSyncOffset.isSet(message_t* msg)
	{
		#warning "return bool"
		return call TimeSyncFlag.get(msg);
	}

	async command uint8_t PacketTimeSyncOffset.get(message_t* msg)
	{
		return call RadioPacket.headerLength(msg) + call RadioPacket.payloadLength(msg) - sizeof(timesync_absolute_t);
	}

	async command void PacketTimeSyncOffset.clear(message_t* msg)
	{	
		call TimeSyncFlag.clear(msg);
	}

	async command void PacketTimeSyncOffset.set(message_t* msg, uint8_t value)
	{
		call TimeSyncFlag.set(msg);
	}

	/*----------------- PacketRSSI -----------------*/

	async command uint8_t PacketRSSI.isSet(message_t* msg)
	{
		#warning "return bool"
		return call RSSIFlag.get(msg);
	}

	async command int8_t PacketRSSI.get(message_t* msg)
	{
		return getMeta(msg)->rssi;
	}

	async command void PacketRSSI.clear(message_t* msg)
	{
		call RSSIFlag.clear(msg);
	}

	async command void PacketRSSI.set(message_t* msg, int8_t value)
	{
		// just to be safe if the user fails to clear the packet
		call TransmitPowerFlag.clear(msg);

		call RSSIFlag.set(msg);
		getMeta(msg)->rssi = value;
	}

	/*----------------- PacketTransmitPower -----------------*/

	async command uint8_t PacketTransmitPower.isSet(message_t* msg)
	{
		#warning "return bool"
		return call TransmitPowerFlag.get(msg);
	}

	async command uint8_t PacketTransmitPower.get(message_t* msg)
	{
		// return getMetadata(msg)->power;
	}

	async command void PacketTransmitPower.clear(message_t* msg)
	{
		call TransmitPowerFlag.clear(msg);
	}

	async command void PacketTransmitPower.set(message_t* msg, uint8_t value)
	{
		call TransmitPowerFlag.set(msg);
		// getMeta(msg)->power = value;
	}

	/*----------------- PacketLinkQuality -----------------*/

	async command uint8_t PacketLinkQuality.isSet(message_t* msg)
	{
		#warning "return bool"
		return TRUE;
	}

	async command uint8_t PacketLinkQuality.get(message_t* msg)
	{
		return getMeta(msg)->lqi;
	}

	async command void PacketLinkQuality.clear(message_t* msg)
	{
	}

	async command void PacketLinkQuality.set(message_t* msg, uint8_t value)
	{
		getMeta(msg)->lqi = value;
	}

	/*----------------- LinkPacketMetadata -----------------*/

	async command uint8_t LinkPacketMetadata.highChannelQuality(message_t* msg)
	{
		#warning "return bool"
		return call PacketLinkQuality.get(msg) > 200;
	}

	  /*----------------- CHANNEL -----------------*/
    tasklet_async command uint8_t RadioState.getChannel()
  	{	
  		channel = *(uint32_t *) CHANNEL_ADDR;
  		if (channel == 0xFF) {
			channel = DEFAULT_RADIO_CHANNEL;
		}
		return channel;
	}

	tasklet_async command error_t RadioState.setChannel(uint8_t c)
	{
		uint32_t *userDataPage = (uint32_t *) 0x0FE00000;
		uint16_t tos_id = *(uint32_t *) TOS_ID_ADDR;
		uint32_t writeData[] = {tos_id, c};
		int8_t status = 0;

		channel = c;
		
		status = MSC_ErasePage(userDataPage);
		if (status != 0) {
			err1("Unable to erase flash: %u", status);
		}

		status = MSC_WriteWord(userDataPage, writeData, sizeof(writeData));
		if (status != 0) {
			err1("Unable to write to flash: %u", status);
		}
		
		signal RadioState.done();
		return SUCCESS;
	}

	tasklet_async command error_t RadioState.turnOff()
	{
		RAIL_Idle(railHandle, RAIL_IDLE_FORCE_SHUTDOWN_CLEAR_FLAGS, false);
		signal RadioState.done();
		return SUCCESS;
	}

	tasklet_async command error_t RadioState.standby()
	{
		#warning "Empty standby()"
		return SUCCESS;
	}

	tasklet_async command error_t RadioState.turnOn()
	{
		channel = *(uint32_t *) CHANNEL_ADDR;
		
		if (channel == 0xFF) {
			channel = DEFAULT_RADIO_CHANNEL;
			warn1("DEFAULT_RADIO_CHANNEL = %u", channel);
		}

		RAIL_Idle(railHandle, RAIL_IDLE, true);
		RAIL_StartRx(railHandle, channel, NULL);
		signal RadioState.done();
		return SUCCESS;
	}

	default tasklet_async event void RadioState.done() { }


    async command error_t PacketAcknowledgements.requestAck(message_t* msg)
	{
		getIeeeHeader(msg)->fcf |= (1 << 5);
		// call Ieee154PacketLayer.setAckRequired(msg, TRUE);

		return SUCCESS;
	}

	async command error_t PacketAcknowledgements.noAck(message_t* msg)
	{
		getIeeeHeader(msg)->fcf &= ~(uint16_t)(1 << 5);
		//call Ieee154PacketLayer.setAckRequired(msg, FALSE);

		return SUCCESS;
	}

  	async command uint8_t PacketAcknowledgements.wasAcked(message_t* msg)
	{
		#warning "return bool"
		return call AckReceivedFlag.get(msg);
	}
}
