/**
* @file RobotPoseDisambiguator.h
*	This file implements a module that provides the robot pose filtered using the GlobalBallEstimation provided by PTracking
*   library (developed by Fabio Previtali).
* @author Fabio Previtali
*/

#include "RobotPoseDisambiguator.h"
#include <Utils/Utils.h>
#include <unistd.h>

// Uncomment if you want to have debug information.
//#define DEBUG_MODE
//#define BHUMAN_ROBOT_POSE

using namespace std;
using namespace PTracking;

MAKE_MODULE(RobotPoseDisambiguator, spqr_modules)

RobotPoseDisambiguator::RobotPoseDisambiguator(){;}

void RobotPoseDisambiguator::update(RobotPoseSpqrFiltered& robotPoseSpqrFiltered)
{

    static Timestamp lastPrintTime;
    static bool isFirstTime = true;
	
    Point2of robotPose;

    if (isFirstTime)
    {
        do
        {
            usleep(100e3);
        }
        while (theRobotInfo.number == 0);
		
        isFirstTime = false;
    }
	
    bool isPrintTime;


    isPrintTime = ((Timestamp() - lastPrintTime).getMs() > 1000.0);

#ifdef BHUMAN_ROBOT_POSE
	robotPoseSpqrFiltered.x = theRobotPose.translation.x();
	robotPoseSpqrFiltered.y = theRobotPose.translation.y();
	robotPoseSpqrFiltered.theta = theRobotPose.rotation;
#endif

    if ((isPrintTime) && (theGameInfo.state == STATE_INITIAL))
    {
        lastPrintTime.setToNow();
		
        cerr << "\033[22;36;1m"<<"[Module::RobotPoseDisambiguator] Robot pose filtered (" << theRobotInfo.number << ") -> ["
             << (robotPoseSpqrFiltered.x / 1000.0) << "," << (robotPoseSpqrFiltered.y / 1000.0) << "," << Utils::rad2deg(robotPoseSpqrFiltered.theta)
             << "]\033[0m" << endl;
    }

#ifndef BHUMAN_ROBOT_POSE
	bool isFlipped = false;

	// The robot is not sufficiently well-localized and is not correctly estimating the ball.
    if ((!theGlobalBallEstimation.isSingleRobotValid) || (theRobotPose.validity <= 0.6))
    {
        robotPoseSpqrFiltered.x = theRobotPose.translation.x();
        robotPoseSpqrFiltered.y = theRobotPose.translation.y();
        robotPoseSpqrFiltered.theta = theRobotPose.rotation;
		
        return;
    }
	
    robotPose.x = theRobotPose.translation.x();
    robotPose.y = theRobotPose.translation.y();
    robotPose.theta = theRobotPose.rotation;
	
    if (((fabs(theGlobalBallEstimation.multiRobotX) > DISTANCE_THRESHOLD_X) && (fabs(theGlobalBallEstimation.multiRobotY) > DISTANCE_THRESHOLD_Y)) &&
        ((fabs(theGlobalBallEstimation.singleRobotX) > DISTANCE_THRESHOLD_X) && (fabs(theGlobalBallEstimation.singleRobotY) > DISTANCE_THRESHOLD_Y)))
    {
        if (((theGlobalBallEstimation.singleRobotX * theGlobalBallEstimation.multiRobotX) < 0.0) &&
            ((theGlobalBallEstimation.singleRobotY * theGlobalBallEstimation.multiRobotY) < 0.0))
        {
            isFlipped = true;
        }
    }
    else if ((fabs(theGlobalBallEstimation.multiRobotX) > DISTANCE_THRESHOLD_X) && (fabs(theGlobalBallEstimation.singleRobotX) > DISTANCE_THRESHOLD_X))
    {
        if ((theGlobalBallEstimation.singleRobotX * theGlobalBallEstimation.multiRobotX) < 0.0)
        {
            isFlipped = true;
        }
    }
    else if ((fabs(theGlobalBallEstimation.multiRobotY) > DISTANCE_THRESHOLD_Y) && (fabs(theGlobalBallEstimation.singleRobotY) > DISTANCE_THRESHOLD_Y))
    {
        if ((theGlobalBallEstimation.singleRobotY * theGlobalBallEstimation.multiRobotY) < 0.0)
        {
            isFlipped = true;
        }
    }
    else if ((theRobotInfo.number == 1) && (robotPose.x > 0)) isFlipped = true;
    else
    {
        // Siamo all'interno del cerchio di centrocampo oppure non è necessario fare il flip.
    }
	
    if (isFlipped)
    {
#ifdef DEBUG_MODE
        if (isPrintTime)
        {
            cerr << "\033[22;36;1m(" << theRobotInfo.number << ") -> singleRobotBallEstimation: [" << (theGlobalBallEstimation.singleRobotX / 1000.0)
                 << "," << (theGlobalBallEstimation.singleRobotY / 1000.0) << "], globalRobotBallEstimation: ["
                 << (theGlobalBallEstimation.multiRobotX / 1000.0) << "," << (theGlobalBallEstimation.multiRobotY / 1000.0) << "]\033[0m" << endl;
				
            cerr << "\033[22;36;1mOld robot pose (" << theRobotInfo.number << ") -> [" << (robotPoseSpqrFiltered.x / 1000.0) << ","
                 << (robotPoseSpqrFiltered.y / 1000.0) << "," << Utils::rad2deg(robotPoseSpqrFiltered.theta) << "]\033[0m" << endl;
        }
#endif
		
        robotPoseSpqrFiltered.x = -robotPose.x;
        robotPoseSpqrFiltered.y = -robotPose.y;
        robotPoseSpqrFiltered.theta = Utils::angNormPiSig(robotPose.theta + Utils::deg2rad(180.0));

		
#ifdef DEBUG_MODE
        if (isPrintTime) cerr << "\033[22;35;1mInverting robot pose...\033[0m" << endl;
#endif
    }
    else
    {
        robotPoseSpqrFiltered.x = robotPose.x;
        robotPoseSpqrFiltered.y = robotPose.y;
        robotPoseSpqrFiltered.theta = robotPose.theta;
    }
	
#ifdef DEBUG_MODE
    if (isPrintTime)
    {
        lastPrintTime.setToNow();
		
        cerr << "\033[22;36;1mRobot pose filtered (" << theRobotInfo.number << ") -> [" << (robotPoseSpqrFiltered.x / 1000.0) << ","
             << (robotPoseSpqrFiltered.y / 1000.0) << "," << Utils::rad2deg(robotPoseSpqrFiltered.theta) << "]\033[0m" << endl;
    }
#endif

#endif
}
