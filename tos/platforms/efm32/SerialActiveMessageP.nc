//$Id: SerialActiveMessageP.nc,v 1.11 2010-06-29 22:07:50 scipio Exp $

/* Copyright (c) 2000-2005 The Regents of the University of California.  
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
 */

/**
 * Sending active messages over the serial port.
 *
 * @author Philip Levis
 * @author Ben Greenstein
 * @date August 7 2005
 *
 */

#include "main.h"
#include "hdlc_lib.h"
#include <Serial.h>
// generic module SerialActiveMessageP () {
module SerialActiveMessageP {
  provides {
    interface AMSend[am_id_t id];
    interface Receive[am_id_t id];
    interface AMPacket;
    interface Packet;
    interface PacketAcknowledgements;
  }
  uses {
    interface Send as SubSend;
    interface Receive as SubReceive;
  }
}
implementation {

  task void rcvdTask();
  task void sentTask();

  #define __MODUUL__ "SerialAMP"
  #define __LOG_LEVEL__ ( LOG_LEVEL_SerialAMP & BASE_LOG_LEVEL )
  #include "log.h"  

  message_t *msgCopy;
  message_t g_msg;
  message_t *g_msg_copy = &g_msg;


  void* getPayload(message_t* msg)
  {
    return ((void*)msg) + sizeof(message_header_t);
  }


  task void rcvdTask() {
    void* payload = getPayload(g_msg_copy);
    g_msg_copy = signal SubReceive.receive(g_msg_copy, payload, 15);
  }


  task void sentTask() {
    signal SubSend.sendDone(msgCopy, SUCCESS);
  }


  void hdlc_sent() @C() @spontaneous() {
    warn1("sntDone");
    post sentTask();
  }


  void hdlcSerial_rcvd(uint8_t *txData, uint8_t payloadLen) @C() @spontaneous() {
    uint8_t i = 0;

    for (i = 0; i < sizeof(message_header_t); i++) {
      g_msg_copy->header[i] = txData[i+1];
    }

    for (i = 0; i < payloadLen; i++) {
      g_msg_copy->data[i] = txData[12+i];
    }
    
    post rcvdTask();
  }


  serial_header_t* ONE getHeader(message_t* ONE msg) {
    return TCAST(serial_header_t* ONE, (uint8_t*)msg + offsetof(message_t, data) - sizeof(serial_header_t));
  }

  serial_metadata_t* getMetadata(message_t* msg) {
    return (serial_metadata_t*)(msg->metadata);
  }
  
  command error_t AMSend.send[am_id_t id](am_addr_t dest,
					  message_t* msg,
					  uint8_t len) {
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t status = 0;
    uint8_t toUart[150];

    serial_header_t* header = getHeader(msg);

    if (len > call Packet.maxPayloadLength()) {
      return ESIZE;
    }

    header->dest = dest;
    // Do not set the source address or group, as doing so
    // prevents transparent bridging. Need a better long-term
    // solution for this.
    //header->src = call AMPacket.address();
    //header->group = TOS_AM_GROUP;
    header->type = id;
    header->length = len;

    // if (uartTx == true) {
    //   warn1("uartTx EBUSY");
    //   return FAIL;
    // }

    for (i = 12; i < 12+len; i++) {
      toUart[i] = msg->data[j];
      j++;
    }
    toUart[1] = len + 12;
    toUart[7] = msg->header[4]; //AM_DESTINATION_HI_POS
    toUart[6] = msg->header[5]; //AM_DESTINATION_LOW_POS
    toUart[9] = msg->header[6]; //AM_SOURCE_HI_POS
    toUart[8] = msg->header[7]; //AM_SOURCE_LOW_POS
    toUart[0] = msg->header[8]; //DATA_LEN
    toUart[4] = msg->header[9]; //AM_PAN_ID_LOW_POS
    toUart[11] = msg->header[10]; // AM_ID_POS

    msgCopy = msg;

    debugb1("rcv %04X->%04X", toUart, len+sizeof(message_header_t), call AMPacket.source(msg), call AMPacket.destination(msg));
    // hdlc_encode(toUart, len+12, 0x45);

    status = osMessageQueuePut(queueHandle, toUart, 0, 0);

    if (status != 0) {
      warn1("osStatus_t = %u", status);
    }

    return SUCCESS;//call SubSend.send(msg, len);
  }

  command error_t AMSend.cancel[am_id_t id](message_t* msg) {
    return call SubSend.cancel(msg);
  }

  command uint8_t AMSend.maxPayloadLength[am_id_t id]() {
    return call Packet.maxPayloadLength();
  }

  command void* AMSend.getPayload[am_id_t id](message_t* m, uint8_t len) {
    return call Packet.getPayload(m, len);
  }
  
  event void SubSend.sendDone(message_t* msg, error_t result) {
    signal AMSend.sendDone[call AMPacket.type(msg)](msg, result);
  }

 default event void AMSend.sendDone[uint8_t id](message_t* msg, error_t result) {
   return;
 }

 default event message_t* Receive.receive[uint8_t id](message_t* msg, void* payload, uint8_t len) {
   return msg;
 }
 
 event message_t* SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
    return signal Receive.receive[call AMPacket.type(msg)](msg, msg->data, len);
  }

  command void Packet.clear(message_t* msg) {
    memset(getHeader(msg), 0, sizeof(serial_header_t));
    return;
  }

  command uint8_t Packet.payloadLength(message_t* msg) {
    serial_header_t* header = getHeader(msg);    
    return header->length;
  }

  command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
    getHeader(msg)->length  = len;
  }
  
  command uint8_t Packet.maxPayloadLength() {
    return TOSH_DATA_LENGTH;
  }
  
  command void* Packet.getPayload(message_t* msg, uint8_t len) {
    if (len > call Packet.maxPayloadLength()) {
      return NULL;
    }
    else {
      return (void * COUNT_NOK(len))msg->data;
    }
  }

  command am_addr_t AMPacket.address() {
    return 0;
  }

  command am_addr_t AMPacket.destination(message_t* amsg) {
    serial_header_t* header = getHeader(amsg);
    return header->dest;
  }

  command am_addr_t AMPacket.source(message_t* amsg) {
    serial_header_t* header = getHeader(amsg);
    return header->src;
  }

  command void AMPacket.setDestination(message_t* amsg, am_addr_t addr) {
    serial_header_t* header = getHeader(amsg);
    header->dest = addr;
  }

  command void AMPacket.setSource(message_t* amsg, am_addr_t addr) {
    serial_header_t* header = getHeader(amsg);
    header->src = addr;
  }
  
  command bool AMPacket.isForMe(message_t* amsg) {
    return TRUE;
  }

  command am_id_t AMPacket.type(message_t* amsg) {
    serial_header_t* header = getHeader(amsg);
    return header->type;
  }

  command void AMPacket.setType(message_t* amsg, am_id_t type) {
    serial_header_t* header = getHeader(amsg);
    header->type = type;
  }

  async command error_t PacketAcknowledgements.requestAck( message_t* msg ) {
    return FAIL;
  }
  async command error_t PacketAcknowledgements.noAck( message_t* msg ) {
    return SUCCESS;
  }
  
  command void AMPacket.setGroup(message_t* msg, am_group_t group) {
    serial_header_t* header = getHeader(msg);
    header->group = group;
  }

  command am_group_t AMPacket.group(message_t* msg) {
    serial_header_t* header = getHeader(msg);
    return header->group;
  }

  command am_group_t AMPacket.localGroup() {
    return TOS_AM_GROUP;
  }

 
  async command bool PacketAcknowledgements.wasAcked(message_t* msg) {
    return FALSE;
  }

}
