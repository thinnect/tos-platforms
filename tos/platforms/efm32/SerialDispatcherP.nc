//$Id: SerialDispatcherP.nc,v 1.10 2010-06-29 22:07:50 scipio Exp $

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
 * This component provides functionality to send many different kinds
 * of serial packets on top of a general packet sending component. It
 * achieves this by knowing where the different packets in a message_t
 * exist through the SerialPacketInfo interface.
 *
 * @author Philip Levis
 * @author Ben Greenstein
 * @date August 7 2005
 *
 */

#include "Serial.h"
#include "main.h"

// generic 
module SerialDispatcherP {
  provides {
    interface Receive[uart_id_t id];
    interface Send[uart_id_t id];
  }
  uses {
    interface SerialPacketInfo as PacketInfo[uart_id_t id];
    // interface ReceiveBytePacket;
    // interface SendBytePacket;
    interface Leds;
  }
}
implementation {

  #define __MODUUL__ "SerialDisP"
  #define __LOG_LEVEL__ ( LOG_LEVEL_SerialDispatcherP & BASE_LOG_LEVEL )
  #include "log.h"

  message_t *msgCopy;
  message_t g_msg;
  message_t *g_msg_copy = &g_msg;

  typedef enum {
    SEND_STATE_IDLE = 0,
    SEND_STATE_BEGIN = 1,
    SEND_STATE_DATA = 2
  } send_state_t;

  enum {
    RECV_STATE_IDLE = 0,
    RECV_STATE_BEGIN = 1,
    RECV_STATE_DATA = 2,
  }; 
  
  typedef struct {
    uint8_t which:1;
    uint8_t bufZeroLocked:1;
    uint8_t bufOneLocked:1;
    uint8_t state:2;
  } recv_state_t;
  
  // We are not busy, the current buffer to use is zero,
  // neither buffer is locked, and we are idle
  recv_state_t receiveState = {0, 0, 0, RECV_STATE_IDLE};
  uint8_t recvType = TOS_SERIAL_UNKNOWN_ID;
  uint8_t recvIndex = 0;

  /* This component provides double buffering. */
  message_t messages[2];     // buffer allocation
  message_t* ONE messagePtrs[2] = { &messages[0], &messages[1]};
  
  // We store a separate receiveBuffer variable because indexing
  // into a pointer array can be costly, and handling interrupts
  // is time critical.
  uint8_t* COUNT_NOK(sizeof(message_t)) receiveBuffer = (uint8_t* COUNT_NOK(sizeof(message_t)))(&messages[0]);

  uint8_t *COUNT_NOK(sizeof(message_t)) sendBuffer = NULL;
  send_state_t sendState = SEND_STATE_IDLE;
  uint8_t sendLen = 0;
  uint8_t sendIndex = 0;
  norace error_t sendError = SUCCESS;
  bool sendCancelled = FALSE;
  uint8_t sendId = 0;


  uint8_t receiveTaskPending = FALSE;
  uart_id_t receiveTaskType = 0;
  uint8_t receiveTaskWhich;
  message_t * ONE_NOK receiveTaskBuf = NULL;
  uint8_t receiveTaskSize = 0;

  uint8_t id = 0;
  uint8_t payloadLen = 0;

  void* getPayload(message_t* msg)
  {
    return ((void*)msg) + sizeof(message_header_t);
  }


  task void sentTask() {
    signal Send.sendDone[sendId](msgCopy, SUCCESS);
  }


  void hdlc_sent_device() @C() @spontaneous() {
    post sentTask();
  }


  task void rcvdTask()
  {
    void* payload = getPayload(g_msg_copy);
    g_msg_copy = signal Receive.receive[id](g_msg_copy, payload, payloadLen);
  }


  void devParam_rcvd(uint8_t *devData, uint8_t length) @C() @spontaneous()
  { 
    uint8_t i = 0;

    payloadLen = length - 3;
    id = devData[0];
    memcpy(g_msg_copy->data, devData+1, length);

    post rcvdTask();
  }

  command error_t Send.send[uint8_t id](message_t* msg, uint8_t len) {
    uint8_t toUart[50] = {0};
    uint8_t offset = 0;
    uint8_t status = 0;
    void* payload = getPayload(msg);
    
    offset = call PacketInfo.offset[id]();

    toUart[1] = len;
    toUart[2] = id; // DISPATCHER_BYTE

    memcpy(toUart+3, msg, len);
    sendId = id;
    msgCopy = msg;
    status = osMessageQueuePut(queueHandle, toUart, 0, 0);

    if (status != 0) {
      warn1("osStatus_t = %u", status);
    }
    
  }

  command uint8_t Send.maxPayloadLength[uint8_t id]() {
    return (sizeof(message_t));
  }

  command void* Send.getPayload[uint8_t id](message_t* m, uint8_t len) {
    if (len > sizeof(message_t)) {
      return NULL;
    }
    else {
      return m;
    }
  }

    
  task void signalSendDone(){
    error_t error;

    sendState = SEND_STATE_IDLE;
    atomic error = sendError;

    if (sendCancelled) error = ECANCEL;
    signal Send.sendDone[sendId]((message_t *)sendBuffer, error);
  }

  command error_t Send.cancel[uint8_t id](message_t *msg){
 //    if (sendState == SEND_STATE_DATA && sendBuffer == ((uint8_t *)msg) &&
	// id == sendId){
 //      call SendBytePacket.completeSend();
 //      sendCancelled = TRUE;
 //      return SUCCESS;
 //    }
 //    return FAIL;
  }

  // async event uint8_t SendBytePacket.nextByte() {
  //   uint8_t b;
  //   uint8_t indx;
  //   atomic {
  //     b = sendBuffer[sendIndex];
  //     sendIndex++;
  //     indx = sendIndex;
  //   }
  //   if (indx > sendLen) {
  //     call SendBytePacket.completeSend();
  //     return 0;
  //   }
  //   else {
  //     return b;
  //   }
  // }
  // async event void SendBytePacket.sendCompleted(error_t error){
  //   atomic sendError = error;
  //   post signalSendDone();
  // }

  bool isCurrentBufferLocked() {
    return (receiveState.which)? receiveState.bufOneLocked : receiveState.bufZeroLocked;
  }

  void lockCurrentBuffer() {
    if (receiveState.which) {
      receiveState.bufOneLocked = 1;
    }
    else {
      receiveState.bufZeroLocked = 1;
    }
  }

  void unlockBuffer(uint8_t which) {
    if (which) {
      receiveState.bufOneLocked = 0;
    }
    else {
      receiveState.bufZeroLocked = 0;
    }
  }
  
  void receiveBufferSwap() {
    receiveState.which = (receiveState.which)? 0: 1;
    receiveBuffer = (uint8_t*)(messagePtrs[receiveState.which]);
  }
  
  // async event error_t ReceiveBytePacket.startPacket() {
  //   error_t result = SUCCESS;
  //   atomic {
  //     if (!isCurrentBufferLocked()) {
  //       // We are implicitly in RECV_STATE_IDLE, as it is the only
  //       // way our current buffer could be unlocked.
  //       lockCurrentBuffer();
  //       receiveState.state = RECV_STATE_BEGIN;
  //       recvIndex = 0;
  //       recvType = TOS_SERIAL_UNKNOWN_ID;
  //     }
  //     else {
  //       result = EBUSY;
  //     }
  //   }
  //   return result;
  // }

  // async event void ReceiveBytePacket.byteReceived(uint8_t b) {
  //   atomic {
  //     switch (receiveState.state) {
  //     case RECV_STATE_BEGIN:
  //       receiveState.state = RECV_STATE_DATA;
  //       recvIndex = call PacketInfo.offset[b]();
  //       recvType = b;
  //       break;
        
  //     case RECV_STATE_DATA:
  //       if (recvIndex < sizeof(message_t)) {
  //         receiveBuffer[recvIndex] = b;
  //         recvIndex++;
  //       }
  //       else {
  //         // Drop extra bytes that do not fit in a message_t.
  //         // We assume that either the higher layer knows what to
  //         // do with partial packets, or performs sanity checks (e.g.,
  //         // CRC).
  //       }
  //       break;
        
  //     case RECV_STATE_IDLE:
  //     default:
  //       // Do nothing. This case can be reached if the component
  //       // does not have free buffers: it will ignore a packet start
  //       // and stay in the IDLE state.
  //     }
  //   }
  // }
  
  // task void receiveTask(){
  //   uart_id_t myType;
  //   message_t *myBuf;
  //   uint8_t mySize;
  //   uint8_t myWhich;
  //   atomic {
  //     myType = receiveTaskType;
  //     myBuf = receiveTaskBuf;
  //     mySize = receiveTaskSize;
  //     myWhich = receiveTaskWhich;
  //   }
  //   mySize -= call PacketInfo.offset[myType]();
  //   mySize = call PacketInfo.upperLength[myType](myBuf, mySize);
  //   myBuf = signal Receive.receive[myType](myBuf, myBuf, mySize);
  //   atomic {
  //     messagePtrs[myWhich] = myBuf;
  //     unlockBuffer(myWhich);
  //     receiveTaskPending = FALSE;
  //   }
  // }

  // async event void ReceiveBytePacket.endPacket(error_t result) {
  //   uint8_t postsignalreceive = FALSE;
  //   atomic {
  //     if (!receiveTaskPending && result == SUCCESS){
  //       postsignalreceive = TRUE;
  //       receiveTaskPending = TRUE;
  //       receiveTaskType = recvType;
  //       receiveTaskWhich = receiveState.which;
  //       receiveTaskBuf = (message_t *)receiveBuffer;
  //       receiveTaskSize = recvIndex;
  //       receiveBufferSwap();
  //       receiveState.state = RECV_STATE_IDLE;
  //     } else {
  //       // we can't deliver the packet, better free the current buffer.
  //       unlockBuffer(receiveState.which);
  //     }
  //   }
  //   if (postsignalreceive){
  //     post receiveTask();
  //   } 
  // }

  default async command uint8_t PacketInfo.offset[uart_id_t id](){
    return 0;
  }
  default async command uint8_t PacketInfo.dataLinkLength[uart_id_t id](message_t *msg,
                                                          uint8_t upperLen){
    return 0;
  }
  default async command uint8_t PacketInfo.upperLength[uart_id_t id](message_t *msg,
                                                       uint8_t dataLinkLen){
    return 0;
  }


  default event message_t *Receive.receive[uart_id_t idxxx](message_t *msg,
                                                         void *payload,
                                                         uint8_t len){
    err1("lol");
    return msg;
  }
  default event void Send.sendDone[uart_id_t idxxx](message_t *msg, error_t error){
    return;
  }

  
}
