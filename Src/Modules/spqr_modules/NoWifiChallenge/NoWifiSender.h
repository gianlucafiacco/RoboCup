#pragma once

#include <iostream>

#include "Tools/Module/Module.h"
#include "Tools/Network/TcpComm.h"

#include "Representations/spqr_representations/NoWifiPacketToSend.h"
#include "Representations/Infrastructure/SPLNoWifiChallenge.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"

#include "RoboCupGameControlData.h"

/**
 * In the Transmitter robot during the NoWifi Challenge is the module that 
 * communicates with the Comms Tester.
 *  
 * Provides the NoWifiPacketToSend that wraps the data received by the 
 * Comms Tester via a TCP packet.
 */
 
MODULE(NoWifiSender,
{,
	REQUIRES(GameInfo),
	REQUIRES(RobotInfo),
	PROVIDES(NoWifiPacketToSend),
});


class NoWifiSender: public NoWifiSenderBase
{
private:
	TcpComm* comm = nullptr;
	//~ bool listening = false;
	
public:

    NoWifiSender(){};
    ~NoWifiSender(){}
    
    void update(NoWifiPacketToSend& noWifiPacketToSend);
};

