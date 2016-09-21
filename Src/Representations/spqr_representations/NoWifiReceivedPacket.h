#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Infrastructure/SPLNoWifiChallenge.h"

/**
 * Represents the data received by the Receiver robot during the
 * no-wifi challenge. This packet is hence delivered to the Comms Tester
 * by the NoWifiReceiver module.
 * 
 * The packet is available when the isPacketAvailable flag is true.
 */

STREAMABLE(NoWifiReceivedPacket,
{
	bool available = false;
	
	struct SPLNoWifiPacket packet,
	
	//~ NoWifiReceivedPacket() = default;
});
