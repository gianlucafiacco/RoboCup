#pragma once

#include <iostream>

#include "Tools/Module/Module.h"
#include "Tools/Network/TcpComm.h"

#include "Representations/spqr_representations/NoWifiReceivedPacket.h"
#include "Representations/spqr_representations/NoWifiPacketRead.h"
#include "Representations/Infrastructure/SPLNoWifiChallenge.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"

#include "RoboCupGameControlData.h"

/**
 * In the Receiver robot during the NoWifi Challenge is the module that 
 * communicates with the Comms Tester.
 *  
 * Requires the NoWifiReceivedPacket and send it to the Comms Tester via 
 * a TCP packet.
 */

MODULE(NoWifiReceiver,
{,
	REQUIRES(GameInfo),
	REQUIRES(RobotInfo),
	REQUIRES(NoWifiReceivedPacket),
	PROVIDES(NoWifiPacketRead),
});


class NoWifiReceiver: public NoWifiReceiverBase
{
private:
	struct SPLNoWifiPacket lastPacket;
	int n_repeated = 0;
	TcpComm* comm = nullptr;
	
public:

    NoWifiReceiver(){};
    ~NoWifiReceiver(){}
    
    void update(NoWifiPacketRead& noWifiPacketRead);
};
