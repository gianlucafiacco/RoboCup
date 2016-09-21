/**
* @file GlobalEstimator.cpp
*	This file implements a module that provides the global ball estimation using Distributed Data Fusion provided
*	by PTracking library (developed by Fabio Previtali).
* @author Fabio Previtali
*/

#include "GlobalEstimator.h"
#include <Platform/SystemCall.h>
#include <Representations/spqr_representations/ConfigurationParameters.h>
#include <Core/Sensors/BasicSensor.h>
#include <UdpSocket.h>
#include <Manfield/configfile/configfile.h>
#include <sys/stat.h>
#include <signal.h>
#include <string.h>
#include <algorithm>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <vector>

#include "Tools/Settings.h"

#define LAMBDA 0

// Uncomment to enable debug prints.
//#define DEBUG_MODE

#define BHUMAN_BALL_PROVIDER

MAKE_MODULE(GlobalEstimator, spqr_modules)

using namespace std;
using namespace PTracking;
using GMapping::ConfigFile;

std::vector<PTracking::ObjectSensorReadingMultiAgent> GlobalEstimator::observationsMultiAgent(0);
PTracking::ObjectSensorReading GlobalEstimator::objectSensorReading;
std::mutex GlobalEstimator::mutex;

GlobalEstimator::GlobalEstimator() : agentId(-1), sendFrames(0)
{
    SPQR::ConfigurationParameters();
	string configDirectory;
	char currentWorkingDirectory[1024];
	int counter;
	
	configDirectory = "";
	
	if (SystemCall::getMode() == SystemCall::simulatedRobot)
	{
		if (getcwd(currentWorkingDirectory,1024)) {;}
		
		configDirectory = currentWorkingDirectory;
		
		configDirectory = configDirectory.substr(0,configDirectory.rfind("/")) + "/";
	}
	else configDirectory = "Config/";

	// retrieve Location
	std::string location = Global::getSettings().location;
	configDirectory += string("Locations/") + location + "/";
	
	initialTimestamp.setToNow();
	initialTimestampMas.setToNow();
	currentTimestamp.setToNow();
	counterResult = 0;
	currentTargetIndex = 0;
	iterationCounter = 0;
	lastCurrentTargetIndex = 0;
	lastTargetIndex = -1;
	maxTargetIndex = 0;
	
	processor.addSensorFilter(&objectParticleFilter);
	
	objectParticleFilter.configure(configDirectory + string("PTracking/parameters.cfg"));
	objectParticleFilter.initFromUniform();
	
	processor.init();
	
	multiAgentProcessor.addSensorFilter(&objectParticleFilterMultiAgent);
	
	objectParticleFilterMultiAgent.configure(configDirectory + string("PTracking/parameters.cfg"));
	
	objectParticleFilterMultiAgent.initFromUniform();
	
	multiAgentProcessor.init();

    objectSensorReading.setSensor(objectParticleFilter.getSensor());
	
	srand(time(0));
	
	counter = 1;
	
	for (int i = 0; i < 256; i += 63)
	{
		for (int j = 0; j < 256; j += 63)
		{
			for (int k = 0; k < 256; k += 63, ++counter)
			{
				colorMap.insert(make_pair(counter,make_pair(i,make_pair(j,k))));
			}
		}
	}
	
#ifdef DEBUG_MODE
	WARN("Particles' number (PF Single-Agent): " << objectParticleFilter.getparticleNumber() << endl);
	WARN("Particles' number (PF Multi-Agent): " << objectParticleFilter.getparticleNumber() << endl);
#endif

	configure(configDirectory);
	
}

string GlobalEstimator::buildHeader() const
{
	stringstream header;
	int differentTypes;
	
	differentTypes = estimatedTargetModelsMultiAgent.size();
	
	if (abs(currentTargetIndex - lastCurrentTargetIndex) > 0) ++differentTypes;
	
	differentTypes += objectParticleFilter.getObservationsMapping().size();
	
	header << agentId << " " << differentTypes << " ";
	
	/// Pay attention: "true" or "false" means respectively whether the point is oriented or not. In the first case you must use Point2ofOnMap.
	//header << "0" << " " << differentType << " AgentPose true #0000ff " << 7 << " " << 2.0 << " ";
	
	for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimatedTargetModelsMultiAgent.begin(); it != estimatedTargetModelsMultiAgent.end(); ++it)
	{
		stringstream s;
		
		const map<int,pair<int,pair<int,int> > >::const_iterator& colorTrack = colorMap.find(it->first);
		
		s << "#" << setw(2) << setfill('0') << std::hex << colorTrack->second.second.second
		  << setw(2) << setfill('0') << std::hex << colorTrack->second.second.first
		  << setw(2) << setfill('0') << std::hex << colorTrack->second.first;
		
		header << "EstimatedTargetModelsWithIdentityMultiAgent false " << s.str() << " " << 6 << " " << (1.5 * agentId) << " ";
	}
	
	if (abs(currentTargetIndex - lastCurrentTargetIndex) > 0)
	{
		header << "TargetPerceptions false #0000ff " << 14 << " " << 2 << " ";
	}
	
	const vector<pair<Point2of,Point2of> >& observationsMapping = objectParticleFilter.getObservationsMapping();
	
	for (vector<pair<Point2of,Point2of> >::const_iterator it = observationsMapping.begin(); it != observationsMapping.end(); ++it)
	{
		header << "ObservationsMapping false #000000 " << 0 << " " <<  2 << " ";
	}
	
	return header.str();
}

void GlobalEstimator::configure(const string& configDirectory)
{
    ConfigFile fCfg;
    string key, section;
    float worldXMax, worldXMin, worldYMax, worldYMin;
    agentId = 1;
	
	if (!fCfg.read(configDirectory + string("PTracking/parameters.cfg")))
	{
		ERR("Error reading file '" << configDirectory + "PTracking/parameters.cfg'. Exiting..."<< endl);
		
		exit(-1);
	}
	
	try
	{
		section = "parameters";
		
		key = "bestParticles";
		bestParticlesNumber = fCfg.value(section,key);
		
		key = "falsePositiveThreshold";
		falsePositiveThreshold = fCfg.value(section,key);
		
		key = "trueNegativeThreshold";
		trueNegativeThreshold = fCfg.value(section,key);
		
		section = "sensor";
		
		key = "maxReading";
		maxReading = fCfg.value(section,key);
		
		section = "location";
		
		key = "worldXMin";
		worldXMin = fCfg.value(section,key);
		
		key = "worldXMax";
		worldXMax = fCfg.value(section,key);
		
		key = "worldYMin";
		worldYMin = fCfg.value(section,key);
		
		key = "worldYMax";
		worldYMax = fCfg.value(section,key);
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	sizeMapX = worldXMax - worldXMin;
	sizeMapY = worldYMax - worldYMin;
	
	if (!fCfg.read(configDirectory + string("PTracking/pviewer.cfg")))
	{
		ERR("Error reading file '" << configDirectory + "PTracking/pviewer.cfg'. Exiting..."<< endl);
		
		exit(-1);
	}
	
	try
	{
		section = "PViewer";
		
		key = "address";
		pViewerAddress = string(fCfg.value(section,key));
		
		key = "port";
		pViewerPort = fCfg.value(section,key);
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
}

/**
 * WARNING: This function must be updated because it does not consider the target identity provided by PTracking.
 */
void GlobalEstimator::estimatedBallConsensus(GlobalBallEstimation& globalBallEstimation)
{
	static Timestamp lastMultiRobotValidity;
	static constexpr float MAXIMUM_ACCEPTABLE_ESTIMATION_VARIANCE_PUT_ME_IN_A_MORE_SUITABLE_HEADER_FILE = 1.5;
	
	vector<pair<Point2of,Point2f> > estimationsToBeFused;
	vector<pair<Point2f,Point2f> > allEstimations;
	multimap<int,int> consensus;
    Point2f consensusEstimation(-1,-1);
	float globalEstimationX, globalEstimationY;
	int index;
	bool isValid = true;
	
    index = -1;

    if (GlobalEstimator::observationsMultiAgent.size() > 1)
    {
        for (vector<ObjectSensorReadingMultiAgent>::const_iterator it = GlobalEstimator::observationsMultiAgent.begin(); it != GlobalEstimator::observationsMultiAgent.end(); ++it)
		{
            const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimation = it->getEstimationsWithModels();

			for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimation.begin(); it2 != estimation.end(); ++it2)
			{
                const Point2f& est = it2->second.first.observation.getCartesian();
                
                if (it2->second.second.mod() < MAXIMUM_ACCEPTABLE_ESTIMATION_VARIANCE_PUT_ME_IN_A_MORE_SUITABLE_HEADER_FILE)
                {
                    allEstimations.push_back(make_pair( est, it2->second.second));
                    lastMultiRobotValidity.setToNow();
                }
            }
		}
		
        int counter, maxCounter = 0;
		
        for (vector<pair<Point2f,Point2f> >::const_iterator it = allEstimations.begin(); it != allEstimations.end(); ++it)
        {
            counter = 0;

            for (vector<pair<Point2f,Point2f> >::const_iterator it2 = allEstimations.begin(); it2 != allEstimations.end(); ++it2)
            {
                if (Utils::isTargetNear(it->first, it2->first, 1.0))
                {
                    counter++;
                }
            }

            if (counter > maxCounter)
            {
                maxCounter = counter;
                consensusEstimation = it->first;
            }
        }
        
        if(consensusEstimation.x == -1)
        {
            isValid = false;
//            INFO("(" << theRobotInfo.number << ") consesusEstimation: => "<<consensusEstimation.x<<", "<<consensusEstimation.y<<endl);
        }
        else
        {
            for (vector<pair<Point2f,Point2f> >::const_iterator it = allEstimations.begin(); it != allEstimations.end(); ++it)
            {
                if (Utils::isTargetNear(it->first, consensusEstimation, 1.0) )
                {
                    Point2of tmpEst;
                    tmpEst.x = it->first.x;
                    tmpEst.y = it->first.y;
                    tmpEst.theta = atan2(it->first.y, it->first.x);
                    estimationsToBeFused.push_back(make_pair(tmpEst, it->second));
                }
            }
        }
		
//		if( consensus.size() > 0 )
//		{
//			index = consensus.rbegin()->second;

//			const pair<ObjectSensorReading::Observation,Point2f>& estimationAgreed = estimatedTargetModelsMultiAgent.at(index);

//			for (vector<pair<Point2f,Point2f> >::const_iterator it = allEstimations.begin(); it != allEstimations.end(); ++it)
//			{
//				if (Utils::isTargetNear(it->first,estimationAgreed.first.observation.getCartesian(),1.0) )
//				{
//					Point2of tmpEst;
//					tmpEst.x = it->first.x;
//					tmpEst.y = it->first.y;
//					tmpEst.theta = atan2(it->first.y, it->first.x);
//					estimationsToBeFused.push_back(make_pair(tmpEst, it->second));
//				}
//			}
//		}

    }
    else if (GlobalEstimator::observationsMultiAgent.size() == 1)
	{
		float minVariance = FLT_MAX;
        int index2 = -1;
        const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimation = GlobalEstimator::observationsMultiAgent.begin()->getEstimationsWithModels();
		
        for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimation.begin(); it2 != estimation.end(); ++it2)
		{
            if (it2->second.second.mod() < minVariance  && it2->second.second.mod() < MAXIMUM_ACCEPTABLE_ESTIMATION_VARIANCE_PUT_ME_IN_A_MORE_SUITABLE_HEADER_FILE)
			{
				minVariance = it2->second.second.mod();
                index2 = it2->first;
			}
		}

		if( index2 == -1 )
		{
            isValid = false;
		}
		else
		{
			Point2of tmpEst;
			tmpEst.x = estimation.at(index2).first.observation.getCartesian().x;
			tmpEst.y = estimation.at(index2).first.observation.getCartesian().y;
			tmpEst.theta = atan2(estimation.at(index2).first.observation.getCartesian().y, estimation.at(index2).first.observation.getCartesian().x);
			estimationsToBeFused.push_back(make_pair(tmpEst, estimation.at(index2).second));
			
            lastMultiRobotValidity.setToNow();
		}
    }
	if (GlobalEstimator::observationsMultiAgent.size() > 0 && isValid)
	{
		float allSigmaX, allSigmaY, varianceX, varianceY;
		float sigmaNormalizationRatioX, sigmaNormalizationRatioY;
		
		allSigmaX = 0.0;
		allSigmaY = 0.0;

		varianceX = 0.0;
		varianceY = 0.0;

//		int varianceXCounter = 0;
//		int varianceYCounter = 0;
		
		for (vector<pair<Point2of,Point2f> >::const_iterator it = estimationsToBeFused.begin(); it != estimationsToBeFused.end(); ++it)
		{
//			if (it->second.x < MAXIMUM_ACCEPTABLE_ESTIMATION_VARIANCE_PUT_ME_IN_A_MORE_SUITABLE_HEADER_FILE)
//			{
//				varianceX += it->second.x;
//				++varianceXCounter;
//			}
//			if (it->second.y < MAXIMUM_ACCEPTABLE_ESTIMATION_VARIANCE_PUT_ME_IN_A_MORE_SUITABLE_HEADER_FILE)
//			{
//				varianceY += it->second.y;
//				++varianceYCounter;
//			}

			allSigmaX += it->second.x;
			allSigmaY += it->second.y;
		}
		
		/// Fusing estimations.
		if (estimationsToBeFused.size() > 1)
		{
			globalEstimationX = 0.0;
			globalEstimationY = 0.0;
			
			sigmaNormalizationRatioX = 0.0;
			sigmaNormalizationRatioY = 0.0;
			
			for (vector<pair<Point2of,Point2f> >::const_iterator it = estimationsToBeFused.begin(); it != estimationsToBeFused.end(); ++it)
			{
				sigmaNormalizationRatioX += (1.0 - (it->second.x / allSigmaX));
				sigmaNormalizationRatioY += (1.0 - (it->second.y / allSigmaY));
			}
			
			sigmaNormalizationRatioX = 1.0 / sigmaNormalizationRatioX;
			sigmaNormalizationRatioY = 1.0 / sigmaNormalizationRatioY;
			
			for (vector<pair<Point2of,Point2f> >::const_iterator it = estimationsToBeFused.begin(); it != estimationsToBeFused.end(); ++it)
			{
				globalEstimationX += (it->first.x * ((1.0 - (it->second.x / allSigmaX)) * sigmaNormalizationRatioX));
				globalEstimationY += (it->first.y * ((1.0 - (it->second.y / allSigmaY)) * sigmaNormalizationRatioY));
			}
			
             allSigmaX /= estimationsToBeFused.size();
             allSigmaY /= estimationsToBeFused.size();


		}
		else
		{
			globalEstimationX = estimationsToBeFused.begin()->first.x;
			globalEstimationY = estimationsToBeFused.begin()->first.y;
			
            allSigmaX = estimationsToBeFused.begin()->second.x;
            allSigmaY = estimationsToBeFused.begin()->second.y;
		}
		
		globalBallEstimation.multiRobotX = globalEstimationX * 1000.0;
		globalBallEstimation.multiRobotY = globalEstimationY * 1000.0;
        globalBallEstimation.multiRobotVariance = Utils::roundN(sqrt((allSigmaX * allSigmaX) + (allSigmaY * allSigmaY)),2);
        
		globalBallEstimation.isMultiRobotValid = (globalBallEstimation.multiRobotVariance < MAXIMUM_ACCEPTABLE_ESTIMATION_VARIANCE_PUT_ME_IN_A_MORE_SUITABLE_HEADER_FILE) ? true : false;
	}
	else
	{
		globalBallEstimation.multiRobotVariance = 100.0;
		globalBallEstimation.isMultiRobotValid = false;
	}
	
	globalBallEstimation.isMultiRobotValid = (Timestamp() - lastMultiRobotValidity).getMs() < 1000;
}

string GlobalEstimator::prepareDataForViewer() const
{
	stringstream streamDataToSend;
	int i;
	
	streamDataToSend << buildHeader();
	
	//streamDataToSend << "1 " << Utils::Point2ofOnMap << " " << Utils::roundN(agentPose.x,2) << " " << Utils::roundN(agentPose.y,2) << " " << Utils::roundN(agentPose.theta,2);
	
	for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimatedTargetModelsMultiAgent.begin(); it != estimatedTargetModelsMultiAgent.end(); ++it)
	{
		streamDataToSend << " 1 " << Utils::Point2fOnMap << " " << Utils::roundN(it->second.first.observation.getCartesian().x,2) << " " << Utils::roundN(it->second.first.observation.getCartesian().y,2);
	}
	
	if (abs(currentTargetIndex - lastCurrentTargetIndex) > 0)
	{
		streamDataToSend << " " << ((lastCurrentTargetIndex > currentTargetIndex) ? ((LAST_N_TARGET_PERCEPTIONS - lastCurrentTargetIndex) + currentTargetIndex)
																				  : (currentTargetIndex - lastCurrentTargetIndex));
	}
	
	i = lastCurrentTargetIndex;
	
	while (i != currentTargetIndex)
	{
		streamDataToSend << " " << Utils::Point2fOnMap << " " << Utils::roundN(targetVector[i].observation.getCartesian().x,2) << " " << Utils::roundN(targetVector[i].observation.getCartesian().y,2);
		
		++i;
		
		if (i == LAST_N_TARGET_PERCEPTIONS) i = 0;
	}
	
	const vector<pair<Point2of,Point2of> >& observationsMapping = objectParticleFilter.getObservationsMapping();
	
	for (vector<pair<Point2of,Point2of> >::const_iterator it = observationsMapping.begin(); it != observationsMapping.end(); ++it)
	{
		streamDataToSend << " 1 " << Utils::Line2dOnMap << " " << Utils::roundN(it->first.x,2) << " " << Utils::roundN(it->first.y,2)
						 << " " << Utils::roundN(it->second.x,2) << " " << Utils::roundN(it->second.y,2);
	}
	
	return streamDataToSend.str();
}

void GlobalEstimator::updateRobotObservations()
{
    if(!theTeammateData.teammates.size()) return;
    for(unsigned int i=0; i<theTeammateData.teammates.size(); ++i)
    {
        static PTracking::ObjectSensorReadingMultiAgent objectSensorReadingMultiAgent;
        objectSensorReadingMultiAgent.setSensor(GlobalEstimator::objectSensorReading.getSensor());
        std::string dataReceived = "Agent " + theTeammateData.teammates.at(i).globalBallEstimation.observations;

        PTracking::AgentPacket ap;

        std::cerr<<"dataReceived: "<<dataReceived<<std::endl;
        if(dataReceived.length() == std::string("Agent ").length()) continue;
        ap.setData(dataReceived.substr(dataReceived.find(" ") + 1));
        objectSensorReadingMultiAgent.setEstimationsWithModels(ap.dataPacket.estimatedTargetModels); //DIO
        objectSensorReadingMultiAgent.setEstimationsTimestamp(ap.dataPacket.particlesTimestamp);

        mutex.lock();
        observationsMultiAgent.push_back(objectSensorReadingMultiAgent);
        mutex.unlock();
    }
}

void GlobalEstimator::update(GlobalBallEstimation& globalBallEstimation)
{

#ifdef BHUMAN_BALL_PROVIDER

    Vector2f single_pose = Transformation::robotToField(Pose2f(theRobotPoseSpqrFiltered.theta,theRobotPoseSpqrFiltered.x,theRobotPoseSpqrFiltered.y), theBallModel.estimate.position);
    globalBallEstimation.singleRobotX = single_pose.x();
    globalBallEstimation.singleRobotY = single_pose.y();
    globalBallEstimation.singleRobotVariance = 1;
    globalBallEstimation.isSingleRobotValid = (theFrameInfo.time > 500) && ((theFrameInfo.time - theBallModel.timeWhenLastSeen) < BALL_SEEN_THRESHOLD);

    globalBallEstimation.multiRobotX = theTeamBallModel.position.x();
    globalBallEstimation.multiRobotY = theTeamBallModel.position.y();
    globalBallEstimation.multiRobotVariance = 1;
    globalBallEstimation.isMultiRobotValid = theTeamBallModel.isValid;

#else

	vector<ObjectSensorReading> observations;
	Point2of robotPose;
	string dataToSend;
	double rho;
	bool ballSeen, isPrintTimeLocal;
	
	currentTimestamp.setToNow();
	
	isPrintTimeLocal = ((currentTimestamp - lastPrintTimeLocal).getMs() > 1000.0);
	
	if (isPrintTimeLocal) lastPrintTimeLocal.setToNow();
	
    robotPose.x = (1-LAMBDA) * (1 / 1000.0) * theRobotPose.translation.x() + (LAMBDA) * (1 / 1000.0) * theRobotPoseSpqrFiltered.x;
    robotPose.y = (1-LAMBDA) * (1 / 1000.0) * theRobotPose.translation.y() + (LAMBDA) * (1 / 1000.0) * theRobotPoseSpqrFiltered.y;
	robotPose.theta = theRobotPose.rotation;
	
#ifdef DEBUG_MODE
	if (isPrintTimeLocal)
	{
		cerr << "\033[22;31;1m[PTracking] RobotPose (" << theRobotInfo.number << ") ->: [" << Utils::roundN(robotPose.x,2) << "," << Utils::roundN(robotPose.y,2)
			 << "," << Utils::roundN(Utils::rad2deg(robotPose.theta),2) << "], validity: " << theRobotPose.validity << "\033[0m" << endl;
	}
#endif

    static int actualObservations = 1;
	
	// The robot is sufficiently well-localized.
	if (theRobotPose.validity > 0.6)
	{
		ballSeen = ((theFrameInfo.time > 500) && ((theFrameInfo.time - theBallModel.timeWhenLastSeen) < BALL_SEEN_THRESHOLD));
		
		if (ballSeen)
		{
			// Considering just the last ball observation (converting it in meters).
			targetVector[0].observation.rho = sqrt(((theBallModel.estimate.position.x() / 1000.0) * (theBallModel.estimate.position.x() / 1000.0)) +
												   ((theBallModel.estimate.position.y() / 1000.0) * (theBallModel.estimate.position.y() / 1000.0)));
			
			targetVector[0].observation.theta = atan2(theBallModel.estimate.position.y() / 1000.0,theBallModel.estimate.position.x() / 1000.0);
			
			rho = sqrt((targetVector[0].observation.getCartesian().x * targetVector[0].observation.getCartesian().x) +
					(targetVector[0].observation.getCartesian().y * targetVector[0].observation.getCartesian().y));
			
			actualObservations = 1;
			
#ifdef DEBUG_MODE
			if (isPrintTimeLocal)
			{
				cerr << "\033[22;32;1m[PTracking] Observation (" << theRobotInfo.number << ") ->: [" << Utils::roundN(targetVector[0].observation.getCartesian().x,2)
						<< "," << Utils::roundN(targetVector[0].observation.getCartesian().y,2) << "]\033[0m" << endl;
			}
#endif
			
			for (int i = 0; i < actualObservations; ++i)
			{
				if ((fabs(targetVector[i].observation.getCartesian().x) > sizeMapX) || (fabs(targetVector[i].observation.getCartesian().y) > sizeMapY)) ballSeen = false;
			}
        }
        GlobalEstimator::objectSensorReading.setObservationsAgentPose(robotPose);
        GlobalEstimator::objectSensorReading.setObservations(targetVector,ballSeen ? actualObservations : 0,0,LAST_N_TARGET_PERCEPTIONS,maxReading);

        observations.push_back(GlobalEstimator::objectSensorReading);

//        return; //BUG DIOOO
		
        updateRobotObservations();
		processor.processReading(robotPose,ballSeen,initialTimestamp,currentTimestamp,observations);
		
		const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimationsWithModel = objectParticleFilter.getEstimationsWithModel();
		
		updateTargetPosition(estimationsWithModel);
		bestParticles = updateBestParticles(estimationsWithModel);

		
		Point2f minVariance(100.0,100.0);

		for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimationsWithModel.begin(); it != estimationsWithModel.end(); ++it)
		{
			if (it->second.second.mod() < minVariance.mod())
			{
				globalBallEstimation.singleRobotX = it->second.first.observation.getCartesian().x * 1000.0;
				globalBallEstimation.singleRobotY = it->second.first.observation.getCartesian().y * 1000.0;
				globalBallEstimation.singleRobotVariance = Utils::roundN(it->second.second.mod(),2);
				globalBallEstimation.isSingleRobotValid = (it->second.second.mod() < 1.5) ? true : false;
				
				minVariance = it->second.second;
			}
		}
#ifdef DEBUG_MODE
		if (isPrintTimeLocal)
		{
			cerr << "\033[22;34;1m[PTracking] SingleRobot (" << theRobotInfo.number << ") -> [" << Utils::roundN(globalBallEstimation.singleRobotX / 1000.0,2)
				 << "," << Utils::roundN(globalBallEstimation.singleRobotY / 1000.0 ,2) << "], validity = " << globalBallEstimation.isSingleRobotValid
				 << ", variance: " << globalBallEstimation.singleRobotVariance << "\033[0m" << endl;
        }
#endif
		
		if (estimatedTargetModels.size() > 0)
		{
            dataToSend = "";
			
			AgentPacket agentPacket;
            agentPacket.dataPacket.ip = "127.0.0.1";
            agentPacket.dataPacket.port = 10005;
            agentPacket.dataPacket.agentPose = robotPose;
			agentPacket.dataPacket.estimatedTargetModels = estimatedTargetModels;
			agentPacket.dataPacket.particlesTimestamp = currentTimestamp.getMsFromMidnight();
			

			dataToSend += agentPacket.toString();

            if (theTeammateData.sendThisFrame && ballSeen)
			{
                ++sendFrames;
                globalBallEstimation.observations = dataToSend;
                lastTimeInformationSent.setToNow();
            }
			
			PTracking::ObjectSensorReadingMultiAgent objectSensorReadingMultiAgent;
			
            objectSensorReadingMultiAgent.setSensor(objectParticleFilter.getSensor());
            objectSensorReadingMultiAgent.setEstimationsWithModels(estimatedTargetModels);
            objectSensorReadingMultiAgent.setEstimationsTimestamp(currentTimestamp.getMsFromMidnight());

            GlobalEstimator::mutex.lock();

            GlobalEstimator::observationsMultiAgent.push_back(objectSensorReadingMultiAgent);

            GlobalEstimator::mutex.unlock();
		}
		else
		{
			globalBallEstimation.singleRobotVariance = 100.0;
			globalBallEstimation.isSingleRobotValid = false;
		}
		

        initialTimestamp = currentTimestamp;
    }
	
	if ((currentTimestamp - initialTimestampMas).getMs() > (2000.0 / SPQR::COORDINATION_INFORMATION_NETWORK_FREQUENCY))
	{
        bool isPrintTimeGlobal;
		
		isPrintTimeGlobal = ((currentTimestamp - lastPrintTimeGlobal).getMs() > 1000.0);
		
		if (isPrintTimeGlobal) lastPrintTimeGlobal.setToNow();
		
		initialTimestampMas = currentTimestamp;
		
        GlobalEstimator::mutex.lock();

        if (GlobalEstimator::observationsMultiAgent.size() > 0)
        {
            if (GlobalEstimator::observationsMultiAgent.size() > (SPQR::COORDINATION_INFORMATION_NETWORK_FREQUENCY * 5))
            {
                GlobalEstimator::observationsMultiAgent.erase(GlobalEstimator::observationsMultiAgent.begin(),GlobalEstimator::observationsMultiAgent.end() - (SPQR::COORDINATION_INFORMATION_NETWORK_FREQUENCY * 5));
            }

            multiAgentProcessor.processReading(GlobalEstimator::observationsMultiAgent);
            estimatedTargetModelsMultiAgent = objectParticleFilterMultiAgent.getEstimationsWithModel();

        }
		
        estimatedBallConsensus(globalBallEstimation);
        GlobalEstimator::observationsMultiAgent.clear();
        GlobalEstimator::mutex.unlock();
		
		
		/*dataToSend = prepareDataForViewer();

		int ret;

		ret = senderSocket.send(dataToSend,InetAddress(pViewerAddress,pViewerPort));
		
		if (ret == -1)
		{
			ERR("Error when sending message to PViewer." << endl);
		}*/
		
#ifdef DEBUG_MODE
		if (isPrintTimeGlobal)
		{
			cerr << "\033[22;37;1m[PTracking] MultiRobot (" << theRobotInfo.number << ") -> [" << Utils::roundN(globalBallEstimation.multiRobotX / 1000.0,2)
				 << "," << Utils::roundN(globalBallEstimation.multiRobotY / 1000.0,2) << "], validity = " << globalBallEstimation.isMultiRobotValid
				 << ", variance = " << globalBallEstimation.multiRobotVariance << "\033[0m" << endl;
		}
#endif
	}

#endif
}

vector<PoseParticleVector> GlobalEstimator::updateBestParticles(const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimationsWithModel)
{
	bestParticles.clear();
	
	for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimationsWithModel.begin(); it != estimationsWithModel.end(); ++it)
	{
		bestParticles.push_back(Utils::samplingParticles(it->second.first.observation.getCartesian(),it->second.first.sigma,bestParticlesNumber));
	}
	
	return bestParticles;
}

void GlobalEstimator::updateTargetPosition(const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimationsWithModel)
{
	estimatedTargetModels.clear();
	
	for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimationsWithModel.begin(); it != estimationsWithModel.end(); it++)
	{
		estimatedTargetModels.insert(*it);
	}
}
