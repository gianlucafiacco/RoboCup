#pragma once

#include "Tools/Enum.h"

class DiveHandle : public Streamable
{
private:
	/** Streaming function       
		* @param in  streaming in ...
		* @param out ... streaming out.
	*/
    void serialize(In* in, Out* out)
    {
        STREAM_REGISTER_BEGIN;
        STREAM(diveTime);
        STREAM(ballProjectionEstimate);
        STREAM(diveType);
        STREAM(rewardAck);
        STREAM_REGISTER_FINISH;
    }

public:
	ENUM(Dive,
   {,
         no_dive,
         lDive,
         rDive,
         lcloseDive,
         rcloseDive,
         stopBall,
    });

    float diveTime;
    float ballProjectionEstimate;
    Dive diveType;

    bool rewardAck;

    /** Constructor */
    DiveHandle() : diveTime(-1.0), ballProjectionEstimate(0.0),
        diveType(no_dive),rewardAck(false) {;}
};
