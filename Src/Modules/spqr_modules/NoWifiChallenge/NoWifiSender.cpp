#include "NoWifiSender.h"

#include <stdio.h>
#include <string.h>

//~ TODO: convertire in positivo 

MAKE_MODULE(NoWifiSender, spqr_modules)

//~ NoWifiSender::NoWifiSender(): comm(TcpComm(0, SPL_NO_WIFI_DEFAULT_TRANSMITTER_ROBOT_PORT))
//~ {
//~ }

void NoWifiSender::update(NoWifiPacketToSend& noWifiPacketToSend)
{
	// The robot number 2 is the sender
	if (theRobotInfo.number == 2) {
		//~ std::cerr << "NoWifiSender ->" << std::endl;
		if (theGameInfo.state == STATE_PLAYING && theRobotInfo.penalty == PENALTY_NONE) {
			//~ std::cerr << "NoWifiSender -> playing no penalty" << std::endl;
			if (comm == nullptr) {
				std::cerr << "NoWifiSender -> comm == nullptr" << std::endl;
				comm = new TcpComm(0, SPL_NO_WIFI_DEFAULT_TRANSMITTER_ROBOT_PORT);
				if (comm != nullptr)
					std::cerr << "NoWifiSender -> connection created" << std::endl;
				else
					std::cerr << "NoWifiSender -> unable to create connection" << std::endl;
			} else {
				//~ std::cerr << "NoWifiSender -> comm != nullptr" << std::endl;
				struct SPLNoWifiPacket packet;
				struct SPLNoWifiHeader header;
				if (comm->receive((unsigned char*) &header, sizeof(header), true)) { // read header
					packet.header = header;
					uint16_t type = (uint16_t) header.type;
					std::cerr << "Received header " << type << std::endl;
					if (type == SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION) {
						struct SPLNoWifiLocationPayload location;
						if (comm->receive((unsigned char*) &location, sizeof(location), true)) { // read location
							std::cerr << "Received location " << location.x << ", " << location.y << std::endl;
							// convert location to positive location
							location.x += 9000/2;
							location.y += 6000/2;
							std::cerr << "Received positive location " << location.x << ", " << location.y << std::endl;
							packet.payload.location = (SPLNoWifiLocationPayload) location;
							noWifiPacketToSend.isPacketAvailable = true;
						} else {
							std::cerr << "No received location " << std::endl;
						}
					} else if (type == SPL_NO_WIFI_PAYLOAD_TYPE_DATA) {
						struct SPLNoWifiDataPayload dataPayload;
						struct SPLNoWifiDataPayloadHeader dataHeader;
						if (comm->receive((unsigned char*) &dataHeader, sizeof(dataHeader), true)) { // read data header
							uint16_t fragOffset = dataHeader.fragmentOffset;
							uint16_t fragLength = dataHeader.fragmentLength;
							std::cerr << "Received data frag " << fragOffset << " -> " << fragLength << std::endl;
							dataPayload.header = dataHeader;
							uint8_t data[fragLength - fragOffset];
							unsigned char* trash;
							comm->receive((unsigned char*) &trash, fragOffset, true); // consider offset
							if (comm->receive((unsigned char*) &data, sizeof(data), true)) { // read actual data
								std::cerr << "Data Received  " << std::endl;
								memcpy(dataPayload.data, data, SPL_NO_WIFI_DATA_PAYLOAD_MAX_LEN);
								packet.payload.data = dataPayload;
								noWifiPacketToSend.isPacketAvailable = true;
							} else {
								std::cerr << "No received data" << std::endl;
							}
						} else {
							std::cerr << "No received data header" << std::endl;
						}
					} else {
						std::cerr << "Wrong header received" << std::endl;
					}
				}
				noWifiPacketToSend.packet = packet;
			}
		} else {
			if (comm != nullptr)
				delete comm;
			comm = nullptr;
			std::cerr << "NoWifiSender -> Socket disconnected" << std::endl;
		}
	}
}

