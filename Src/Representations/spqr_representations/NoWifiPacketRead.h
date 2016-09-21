#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(NoWifiPacketRead,
{,
	(bool) (false) packetRead,
	//~ NoWifiPacketRead() = default;
});
