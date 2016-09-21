#include "NoWifiDummyRecognizer.h"

//#define LOCATION

MAKE_MODULE(NoWifiDummyRecognizer, spqr_modules)

NoWifiDummyRecognizer::NoWifiDummyRecognizer()
{
}

void NoWifiDummyRecognizer::update(NoWifiReceivedPacket& noWifiReceivedPacket) 
{
	// The robot number 1 is the receiver
	if (theRobotInfo.number == 1 && theGameInfo.state == STATE_PLAYING  
			&& theRobotInfo.penalty == PENALTY_NONE) {
#ifdef LOCATION
		struct SPLNoWifiPacket packet;
		struct SPLNoWifiHeader header;
		header.type = (int16_t) SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION;
		packet.header = (SPLNoWifiHeader) header;
		struct SPLNoWifiLocationPayload location;
		location.x = (int16_t) 666;
		location.y = (int16_t) 333;
		packet.payload.location = (SPLNoWifiLocationPayload) location;
#else
		struct SPLNoWifiPacket packet;
		struct SPLNoWifiHeader header;
		header.type = (int16_t) SPL_NO_WIFI_PAYLOAD_TYPE_DATA;
		packet.header = (SPLNoWifiHeader) header;
		struct SPLNoWifiDataPayload payload;
		payload.header.fragmentOffset = 0;
		payload.header.fragmentLength = SPL_NO_WIFI_DATA_PAYLOAD_MAX_LEN;
		for (int i=0; i<SPL_NO_WIFI_DATA_PAYLOAD_MAX_LEN; i++)
			payload.data[i] = 9;
		packet.payload.data = payload;
#endif
		
		noWifiReceivedPacket.packet = packet;
		noWifiReceivedPacket.available = true;
		std::cerr << "Packet produced" << std::endl;
	} else {
		// If the packet has already been read it sould not be read anymore
		if (theNoWifiPacketRead.packetRead) {
			noWifiReceivedPacket.available = false;
			std::cerr << "Packet no more available" << std::endl;
		}
	}
}
