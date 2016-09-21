#include "NoWifiReceiver.h"

#include "Representations/Infrastructure/SPLNoWifiChallenge.h"

//~ TODO: 	Mandare pacchetti di continuo (max 5?)?!
//~ 		

using namespace std;

MAKE_MODULE(NoWifiReceiver, spqr_modules)

//~ NoWifiReceiver::NoWifiReceiver(): comm(nullptr)
//~ {
	//~ if (comm.connected())
		//~ std::cerr << "NoWifiReceiver -> connection to tester succesful" << std::endl;
	//~ else
		//~ std::cerr << "NoWifiReceiver -> connection to tester failed" << std::endl;
//~ }

void NoWifiReceiver::update(NoWifiPacketRead& noWifiPacketRead)
{
	// The robot number 1 is the receiver
	if (theRobotInfo.number == 1) {
		//~ std::cerr << "State " << theGameInfo.getStateAsString() << std::endl;
		if (theGameInfo.state == STATE_PLAYING && theRobotInfo.penalty == PENALTY_NONE) {
			if (comm == nullptr) {
				comm = new TcpComm(SPL_NO_WIFI_COMMS_TESTER_ADDR, SPL_NO_WIFI_COMMS_TESTER_PORT);
				if (comm != nullptr)
					std::cerr << "NoWifiReceiver -> connection created" << std::endl;
				else
					std::cerr << "NoWifiReceiver -> unable to create connection" << std::endl;
			}
			if (comm->connected()) {
				std::cerr << "NoWifiReceiver -> connected" << std::endl;
				struct SPLNoWifiPacket packet;
				if (theNoWifiReceivedPacket.available) {
					packet = theNoWifiReceivedPacket.packet;
					noWifiPacketRead.packetRead = true;
					std::cerr << "NoWifiReceiver : new packet received" << std::endl;
				} else if (++n_repeated < 4) {
					packet = lastPacket;
					std::cerr << "NoWifiReceiver : repeat packet (" << n_repeated << ") " << std::endl;
				} else return; // Send nothing
				
				uint16_t head = packet.header.type;
				if (comm->send((const unsigned char*) &head, sizeof(head))) { 
					if (head == SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION) {
						uint16_t x = packet.payload.location.x;
						uint16_t y = packet.payload.location.y;
						// convert to have the field center in (0, 0)
						x -= 9000/2;
						y -= 6000/2;
						
						if (comm->send((const unsigned char*) &x, sizeof(x)) 
								&& comm->send((const unsigned char*) &y, sizeof(y))) {
							std::cerr << "NoWifiReceiver : " << "data correctly sent" << std::endl;
						} else {
							noWifiPacketRead.packetRead = false;
							std::cerr << "NoWifiReceiver : " << "data not correctly sent" << std::endl;
						}
					} else {
						uint16_t offset = packet.payload.data.header.fragmentOffset;
						uint16_t length = packet.payload.data.header.fragmentLength;
						uint8_t data[SPL_NO_WIFI_DATA_PAYLOAD_MAX_LEN];
						memcpy(data, packet.payload.data.data, SPL_NO_WIFI_DATA_PAYLOAD_MAX_LEN);
						
						if (comm->send((const unsigned char*) &offset, sizeof(offset)) 
								&& comm->send((const unsigned char*) &length, sizeof(length))
								&& comm->send((const unsigned char*) &data, sizeof(data))) {
							std::cerr << "NoWifiReceiver : " << "data correctly sent" << std::endl;
						} else {
							noWifiPacketRead.packetRead = false;
							std::cerr << "NoWifiReceiver : " << "data not correctly sent" << std::endl;
						}
					}
				} else {
					noWifiPacketRead.packetRead = false;
					std::cerr << "NoWifiReceiver : " << "data not correctly sent" << std::endl;
				}
			} else {
				noWifiPacketRead.packetRead = false;
			}
		} else {
			if (comm != nullptr)
				delete comm;
			comm = nullptr;
			std::cerr << "NoWifiReceiver -> Socket disconnected" << std::endl;
		}
	}	
}
