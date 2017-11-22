#ifndef __SLRADIO_H__
#define __SLRADIO_H__

#include <RadioConfig.h>
#include <TinyosNetworkLayer.h>
#include <Ieee154PacketLayer.h>
#include <ActiveMessageLayer.h>
#include <MetadataFlagsLayer.h>
#include <SLDriverLayer.h>
#include <TimeStampingLayer.h>
#include <LowPowerListeningLayer.h>
#include <PacketLinkLayer.h>

//#define TFRAMES_ENABLED

typedef nx_struct slpacket_header_t
{
	nxle_uint8_t length;
#ifndef IEEE154BARE_ENABLED
	ieee154_simple_header_t ieee154;
#ifndef TFRAMES_ENABLED
	network_header_t network;
#endif
#ifndef IEEE154FRAMES_ENABLED
	activemessage_header_t am;
#endif
#endif
} slpacket_header_t;

typedef nx_struct slpacket_footer_t
{
	// the time stamp is not recorded here, time stamped messaged cannot have max length
} slpacket_footer_t;

typedef struct slpacket_metadata_t
{
#ifdef LOW_POWER_LISTENING
	lpl_metadata_t lpl;
#endif
#ifdef PACKET_LINK
	link_metadata_t link;
#endif
	timestamp_metadata_t timestamp;
	flags_metadata_t flags;
	sl_metadata_t sl;
} slpacket_metadata_t;

#endif//__SLRADIO_H__
