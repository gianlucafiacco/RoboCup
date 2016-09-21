
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Infrastructure/SPLNoWifiChallenge.h"

/**
 * Represents the data to transmit by the Transmitter robot during the
 * no-wifi challenge. This packet is provided by the NoWifiSender module.
 * 
 * The packet is available when the isPacketAvailable flag is true.
 */

STREAMABLE(NoWifiPacketToSend,
{
	bool isPacketAvailable = false;
	
	struct SPLNoWifiPacket packet,
	
	//~ NoWifiPacketToSend() = default;
});
