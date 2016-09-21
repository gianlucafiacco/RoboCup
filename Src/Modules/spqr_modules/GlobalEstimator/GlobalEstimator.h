/**
* @file GlobalEstimator.h
*	This file declares a module that provides the global ball estimation using Distributed Data Fusion provided
*	by PTracking library (developed by Fabio Previtali).
* @author Fabio Previtali
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/spqr_representations/GlobalBallEstimation.h"
//#include "Representations/spqr_representations/EQualityNetworkEstimation.h"
#include "Representations/spqr_representations/RobotPoseSpqrFiltered.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include <Core/Processors/Processor.h>
#include <Representations/Infrastructure/TeamInfo.h>
#include <Core/Processors/MultiAgentProcessor.h>
#include <Utils/AgentPacket.h>
#include <mutex>

#include "Representations/spqr_representations/OurDefinitions.h"

MODULE(GlobalEstimator,
{,
 REQUIRES(GameInfo),
 REQUIRES(RobotInfo),
 REQUIRES(OwnTeamInfo),
 REQUIRES(RobotPose),
 REQUIRES(BallModel),
 REQUIRES(TeamBallModel),
 REQUIRES(FrameInfo),
 REQUIRES(TeammateData),
 //    REQUIRES(EQualityNetworkEstimation),
 USES(RobotPoseSpqrFiltered),
 PROVIDES(GlobalBallEstimation),
       });

class GlobalEstimator : public GlobalEstimatorBase
{
private:
    static const unsigned int BALL_SEEN_THRESHOLD		= 500;
    static const int LAST_N_TARGET_PERCEPTIONS			= 1;

    std::map<int,std::pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f> > estimatedTargetModelsMultiAgent;
    std::map<int,std::pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f> > estimatedTargetModels;
    std::vector<PoseParticleVector> bestParticles;
    std::map<int,std::pair<int,std::pair<int,int> > > colorMap;
    PTracking::Processor processor;
    PTracking::MultiAgentProcessor multiAgentProcessor;
    PTracking::ObjectParticleFilter objectParticleFilter;
    PTracking::ObjectParticleFilterMultiAgent objectParticleFilterMultiAgent;
    PTracking::ObjectSensorReading::Observation targetVector[LAST_N_TARGET_PERCEPTIONS];
    PTracking::Point2of agentPose;
    PTracking::Timestamp currentTimestamp, initialTimestamp, initialTimestampMas, lastPrintTimeGlobal, lastPrintTimeLocal, lastTimeInformationSent;
    std::string pViewerAddress;
    float falsePositiveThreshold, maxReading, sizeMapX, sizeMapY, trueNegativeThreshold;
    int agentId, bestParticlesNumber, counterResult, currentTargetIndex, iterationCounter, lastCurrentTargetIndex, lastTargetIndex, maxTargetIndex, pViewerPort;


    static void interruptCallback(int);

    std::string buildHeader() const;
    void configure(const std::string&);
    void estimatedBallConsensus(GlobalBallEstimation&);
    void init();
    std::string prepareDataForViewer() const;
    std::vector<PoseParticleVector> updateBestParticles(const std::map<int,std::pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f> >&);
    void updateTargetPosition(const std::map<int,pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f> >&);
    void updateTargetVector(PTracking::ObjectSensorReading&);
    void updateTargetVector(std::stringstream&);
    void updateRobotObservations();

    int sendFrames;

public:

    static PTracking::ObjectSensorReading objectSensorReading;
    static std::mutex mutex;
    static std::vector<PTracking::ObjectSensorReadingMultiAgent> observationsMultiAgent;

    GlobalEstimator();

    void update(GlobalBallEstimation& globalBallEstimation);
};
