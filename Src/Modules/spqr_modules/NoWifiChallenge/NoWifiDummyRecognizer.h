#pragma once

#include "Representations/spqr_representations/NoWifiReceivedPacket.h"
#include "Representations/spqr_representations/NoWifiPacketRead.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/Module/Module.h"

#include <iostream>



MODULE(NoWifiDummyRecognizer,
{,
	REQUIRES(GameInfo),
	REQUIRES(RobotInfo),
	USES(NoWifiPacketRead),
	PROVIDES(NoWifiReceivedPacket),
});

class NoWifiDummyRecognizer: public NoWifiDummyRecognizerBase
{
private:
	
public:

    NoWifiDummyRecognizer();
    ~NoWifiDummyRecognizer(){};
    
    void update(NoWifiReceivedPacket& noWifiReceivedPacket);
};
