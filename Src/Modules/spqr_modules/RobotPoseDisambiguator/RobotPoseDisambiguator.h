/**
* @file RobotPoseDisambiguator.h
*	This file declares a module that provides the robot pose filtered using the GlobalBallEstimation provided by PTracking
*   library (developed by Fabio Previtali).
* @author Fabio Previtali
*/

#pragma once

#include "Tools/Module/Module.h"
#include <Representations/Infrastructure/GameInfo.h>
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"

#include "Representations/spqr_representations/GlobalBallEstimation.h"
#include "Representations/spqr_representations/RobotPoseSpqrFiltered.h"

#include <Core/Processors/Processor.h>
#include <Core/Processors/MultiAgentProcessor.h>
#include <Utils/AgentPacket.h>

MODULE(RobotPoseDisambiguator,
{,
	REQUIRES(GameInfo),
	REQUIRES(RobotInfo),
	REQUIRES(RobotPose),
	REQUIRES(GlobalBallEstimation),
	PROVIDES(RobotPoseSpqrFiltered),
});

class RobotPoseDisambiguator : public RobotPoseDisambiguatorBase
{
	private:
		static const int DISTANCE_THRESHOLD_X = 500;
		static const int DISTANCE_THRESHOLD_Y = 500;
		
	public:
		RobotPoseDisambiguator();
		
		void update(RobotPoseSpqrFiltered& robotPoseSpqrFiltered);
};
