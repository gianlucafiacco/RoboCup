#pragma once
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(GlobalBallEstimation,
{
    public:
       std::string observations,
       (float)(0.f) singleRobotX,			/** The X position of the ball estimated by this robot on the field (global frame) */
       (float)(0.f) singleRobotY,			/** The Y position of the ball estimated by this robot on the field (global frame) */
       (float)(100.f) singleRobotVariance,	/** The variance of the ball estimation performed by this robot **/
       (bool)(false) isSingleRobotValid,	/** True means that the estimation can be used, false otherwise **/

       (float)(0.f) multiRobotX,			/** The X position of the ball estimated by all robots on the field (global frame) */
       (float)(0.f) multiRobotY,			/** The Y position of the ball estimated by all robots on the field (global frame) */
       (float)(100.f) multiRobotVariance,	/** The variance of the ball estimation performed by the whole robot **/
       (bool)(false) isMultiRobotValid,		/** True means that the estimation can be used, false otherwise **/

       GlobalBallEstimation() = default;
});


STREAMABLE(GlobalBallEstimationCompressed,
{
    public:
        GlobalBallEstimationCompressed() = default;
        GlobalBallEstimationCompressed(const GlobalBallEstimation& globalBallEstimation);
        operator GlobalBallEstimation() const,
        (std::string) observations,
});
