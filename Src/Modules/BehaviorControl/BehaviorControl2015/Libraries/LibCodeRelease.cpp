/**
* @file LibCodeRelease.cpp
*/

#include "../LibraryBase.h"

//#define __LOG
#define TIME_STAMP 2000 // [ms]

using namespace std;	
//~ namespace little_endian_io {

//~ }

namespace Behavior2015
{
#include "LibCodeRelease.h"
#include <fstream>
#include <iostream>
#include <cmath>

LibCodeRelease::LibCodeRelease(): goalie_displacement(300.f),
    angleToGoal(0.f), angleToMyGoal(0.f), kickAngle(0.f), correctionKickAngle(0.f), ballOutOnLeft(false), diveBool(false)
{
    SPQR::ConfigurationParameters();
#ifdef __LOG
    dump.open("/home/robocup/Desktop/dataset/dump.txt", std::fstream::out | std::fstream::app);
    opponent_robot_poses_dump.open("/home/robocup/Desktop/dataset/opponent_robot_poses_dump.txt", std::fstream::out | std::fstream::app);
    own_robot_poses_dump.open("/home/robocup/Desktop/dataset/own_robot_poses_dump.txt", std::fstream::out | std::fstream::app);
#endif
    stamp.setToNow();

    kickMap.insert(std::make_pair("forwardKick",new KickParameters("forwardKick",3000.f,Angle::fromDegrees(0.f))));
    kickMap.insert(std::make_pair("sideKick",new KickParameters("sideKick",2000.f,Angle::fromDegrees(90.f))));
    kickMap.insert(std::make_pair("backKick",new KickParameters("backKick",1000.f,Angle::fromDegrees(180.f))));
    kickMap.insert(std::make_pair("extSideKick",new KickParameters("extSideKick",500.f,Angle::fromDegrees(80.f))));
}

void LibCodeRelease::preProcess()
{
    if(theRobotPoseSpqrFiltered.x > 0.f)
    {
        if( (libCodeRelease.timeSinceBallWasSeen()-5) % 2 ==0) penaltyAngle = (theRobotPose.inverse() * Vector2f (theFieldDimensions.xPosOpponentGroundline, 600.f)).angle();
        else penaltyAngle = (theRobotPose.inverse() * Vector2f (theFieldDimensions.xPosOpponentGroundline, -600.f)).angle();
    }
    else
    {
        if( (libCodeRelease.timeSinceBallWasSeen()-5) % 2 ==0) penaltyAngle = (theRobotPose.inverse() * Vector2f (-theFieldDimensions.xPosOpponentGroundline, 600.f)).angle();
        else penaltyAngle = (theRobotPose.inverse() * Vector2f (-theFieldDimensions.xPosOpponentGroundline, -600.f)).angle();
    }


    angleToGoal = (theRobotPose.inverse() * Vector2f (theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    angleToMyGoal = (theRobotPose.inverse() * Vector2f (-theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    kickAngle = computeKickAngle();
    correctionKickAngle = correctionAngle(kickAngle);
    ballOutOnLeft = sideWhenLastTimeBallWasSeen();
}

void LibCodeRelease::postProcess()
{
#ifdef __LOG
    if(theGameInfo.state == STATE_PLAYING)
    {
        if ((PTracking::Timestamp() - stamp).getMs() > TIME_STAMP)
        {
            saveData();
            stamp.setToNow();
        }
    }
#endif
}

void LibCodeRelease::saveData()
{
    if( opponent_robot_poses_dump.is_open() && theRobotInfo.number == 1 && theOwnTeamInfo.teamColor == TEAM_RED /*&& log conditions*/)
    {
        for(unsigned int r=0; r<theSpqrDWKcombiner.robots_poses.size(); ++r)
        {
            opponent_robot_poses_dump
                    << -theSpqrDWKcombiner.robots_poses.at(r).translation.x() << " "
                    << -theSpqrDWKcombiner.robots_poses.at(r).translation.y() << " ";

            float ra = std::fmod((double) (Angle::fromDegrees(180)+theSpqrDWKcombiner.robots_poses.at(r).rotation),
                                (double) 2 * M_PI);

            if (ra > M_PI) ra = -(2 * M_PI - ra);
            else if (ra < -M_PI) ra = 2 * M_PI + ra;

            opponent_robot_poses_dump << ra << " ";
        }
        opponent_robot_poses_dump << std::endl;
    }

    if( own_robot_poses_dump.is_open() && theRobotInfo.number == 1 && theOwnTeamInfo.teamColor == TEAM_BLUE /*&& log conditions*/)
    {
        for(unsigned int r=0; r<theSpqrDWKcombiner.robots_poses.size(); ++r)
        {
            own_robot_poses_dump
                    << theSpqrDWKcombiner.robots_poses.at(r).translation.x() << " "
                    << theSpqrDWKcombiner.robots_poses.at(r).translation.y() << " "
                    << theSpqrDWKcombiner.robots_poses.at(r).rotation << " ";
        }
        own_robot_poses_dump << std::endl;
    }

    if( dump.is_open() && theRobotInfo.number == 1 && theOwnTeamInfo.teamColor == TEAM_BLUE /*&& log conditions*/)
        dump << theSpqrDWKcombiner.estimated_ball_global.x()<<" "<<theSpqrDWKcombiner.estimated_ball_global.y() <<
                  " " <<theBallModel.estimate.velocity.x()<<" "<< theBallModel.estimate.velocity.y() << std::endl;

}

std::string LibCodeRelease::kickToPerform()
{
    std::map<std::string,KickParameters*>::iterator updater = kickMap.begin();
    float scoreKF, scoreSK, scoreBK, scoreEXT;

    // compute distance score to goal (quality)
#define POSITIONING_MAX_VALUE 90
#define POSITIONING_MIDDLE_VALUE 75
#define POSITIONING_MIN_VALUE 60

    scoreKF = POSITIONING_MAX_VALUE;
    if(theRobotPoseSpqrFiltered.x > 0)                                                       // divide the field in 9 cells with a "log approach"
    {
        if(std::abs(theRobotPoseSpqrFiltered.y) > (0.5f*SPQR::FIELD_DIMENSION_Y))            // divide along the y-coord
        {
            if(theRobotPoseSpqrFiltered.x > 0.66*SPQR::FIELD_DIMENSION_X)
            {   // A
                scoreBK = POSITIONING_MAX_VALUE;
                scoreEXT = .0f;
                // SK 9X6
                //scoreSK = POSITIONING_MIN_VALUE;
                // SK 4X3
                scoreSK = POSITIONING_MIDDLE_VALUE;
            }
            else
            {   // C
                scoreBK = POSITIONING_MIDDLE_VALUE;
                scoreEXT = .0f;
                // SK 9X6
                //scoreSK = POSITIONING_MIN_VALUE;
                // SK 4X3
                scoreSK = POSITIONING_MIDDLE_VALUE;
            }
        }
        else
        {
            if(theRobotPoseSpqrFiltered.x > 0.66*SPQR::FIELD_DIMENSION_X)
            {   // B
                scoreBK = POSITIONING_MIDDLE_VALUE;
                scoreEXT = .0f;
                scoreSK = POSITIONING_MAX_VALUE;
                if(((SPQR::FIELD_DIMENSION_X - theRobotPoseSpqrFiltered.x) < 400.0f) &&  (std::abs(theRobotPoseSpqrFiltered.y) < 700) )
                    scoreEXT = POSITIONING_MAX_VALUE;
            }
            else
            {   // D
                scoreBK = POSITIONING_MIN_VALUE;
                scoreEXT = .0f;
                // SK 9X6
                //scoreSK = POSITIONING_MIN_VALUE;
                // SK 4X3
                scoreSK = POSITIONING_MIDDLE_VALUE;
            }
        }
    }
    else
    {           // E
        scoreBK =  POSITIONING_MIN_VALUE;
        scoreEXT = .0f;
        scoreSK = POSITIONING_MIDDLE_VALUE;
        if(theRobotPoseSpqrFiltered.x > -0.45f*SPQR::FIELD_DIMENSION_X)
            scoreBK = POSITIONING_MIDDLE_VALUE;
    }

    for (; updater!=kickMap.end(); ++updater)
    {
        if(updater->first == "forwardKick")
            updater->second->score =  scoreKF + (100-16.0f*std::abs(std::abs(angleToGoal-std::abs(correctionKickAngle))-updater->second->angle));
        else if(updater->first == "sideKick")
            updater->second->score =  scoreSK + (100-16.0f*std::abs(std::abs(angleToGoal-std::abs(correctionKickAngle))-updater->second->angle));
        else if(updater->first == "backKick")
            updater->second->score =  scoreBK + (100-16.0f*std::abs(std::abs(angleToGoal-std::abs(correctionKickAngle))-updater->second->angle));
        else if(updater->first == "extSideKick")
            updater->second->score =  scoreEXT + (100-16.0f*std::abs(std::abs(angleToGoal-std::abs(correctionKickAngle))-updater->second->angle));
    }
    std::string tmpid = "";
    float tmpscore = .0f;
    std::map<std::string,KickParameters*>::iterator checker = kickMap.begin();
    for (; checker!=kickMap.end(); ++checker)
    {
        if(checker->second->score > tmpscore)
        {
            tmpscore = checker->second->score;
            tmpid = checker->second->id;
        }
    }
    return tmpid;
}

int LibCodeRelease::timeSinceBallWasSeen()
{
    return theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
}

bool LibCodeRelease::between(float value, float min, float max)
{
    return value >= min && value <= max;
}

float LibCodeRelease::norm(float x, float y)
{
    return sqrt((x*x) + (y*y));
}

bool LibCodeRelease::ballIsInGame()
{
    if( norm(theGlobalBallEstimation.singleRobotX, theGlobalBallEstimation.singleRobotY) > 300.f)
        return true;
    else
        return false;
}

bool LibCodeRelease::isBallInKickAwayRange()
{
    if( theBallModel.estimate.position.norm() < SPQR::GOALIE_KICK_AWAY_RANGE )
        return true;
    else
        return false;
}

bool LibCodeRelease::isBallInCoverRange()
{
    if( theBallModel.estimate.position.norm() < SPQR::GOALIE_KICK_AWAY_RANGE * 3 )
        return true;
    else
        return false;
}

float LibCodeRelease::angleToTarget(float x, float y)
{
    return glob2Rel(x, y).translation.angle();
}

float LibCodeRelease::computeKickAngle()
{
    Vector2f leftOppGoalPost  = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal);
    Vector2f rightOppGoalPost = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoal);

    return 0.5*(std::abs(angleToTarget(leftOppGoalPost.x(),leftOppGoalPost.y())
                         -angleToTarget(rightOppGoalPost.x(),rightOppGoalPost.y())));
}

float LibCodeRelease::correctionAngle(float _kickAngle)
{
    if (norm(theRobotPoseSpqrFiltered.x - theFieldDimensions.xPosOpponentGroundline, theRobotPoseSpqrFiltered.y) > 0.3*SPQR::FIELD_DIMENSION_X)
        return  0;
    else
        return 0.5*_kickAngle;
}

bool LibCodeRelease::sideWhenLastTimeBallWasSeen()
{
    if(timeSinceBallWasSeen() < 300)
    {
        if(theBallModel.estimate.position.y() < .0f)
            return true;
        else
            return false;
    }
    else return ballOutOnLeft;
}

bool LibCodeRelease::isTheAreaCleanFromOpponents(float radius)
{
    for (uint i = 0; i < theSpqrDWKcombiner.opponents.size(); ++i)
    {
        Vector2f(theSpqrDWKcombiner.opponents.at(i).estimated_pose.translation);
        if ( (theSpqrDWKcombiner.opponents.at(i).estimated_pose.translation - Vector2f( theRobotPoseSpqrFiltered.x, theRobotPoseSpqrFiltered.y)).norm() < radius)
            return false;
    }
    return true;
}


bool LibCodeRelease::isValueBalanced(float currentValue, float target, float bound)
{
    float minErr = currentValue - (target - bound);
    float maxErr = currentValue - (target + bound);

    if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
        return true;
    else
        return false;
}

bool LibCodeRelease::isGoalieInStartingPosition()
{

    if( isValueBalanced(theRobotPoseSpqrFiltered.x, SPQR::GOALIE_BASE_POSITION_X+1000, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
            isValueBalanced(theRobotPoseSpqrFiltered.y, SPQR::GOALIE_BASE_POSITION_Y+1000, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
        return true;
    else
        return false;
}

bool LibCodeRelease::isGoalieInAngle()
{
    //~ if(isBallInCoverRange())
		//~ if (between(angleToTarget(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), 
				//~ Angle::fromDegrees(-10.f), 
				//~ Angle::fromDegrees(10.f) ) )
			//~ return true;
		//~ else
			//~ return false;
    //~ else
		if (between(theRobotPoseSpqrFiltered.theta, Angle::fromDegrees(-10.f), Angle::fromDegrees(10.f) ))
			return true;
		else
			return false;
}

// AreaX is between -4500 and -3900, areaY is between -1100 and 1100
bool LibCodeRelease::isGoalieInArea()
{
	if (between(theRobotPoseSpqrFiltered.x, -4500, -3900) && between(theRobotPoseSpqrFiltered.y, -1100, 1100))
		return true;
	else 
		return false;
}

// AreaX is between -4500 and -3900, areaY is between -1100 and 1100
bool LibCodeRelease::isBallInArea()
{
	Pose2f gloBall = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
	if (between(gloBall.translation.x(), -4500, -3900) && between(gloBall.translation.y(), -1100, 1100))
		return true;
	else 
		return false;
}

float LibCodeRelease::getGoalieCoverAngleDisplacement()
{
	//~ if (isBallInCoverRange())
		//~ return -angleToTarget(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
	//~ else
		return theRobotPoseSpqrFiltered.theta;
}

bool LibCodeRelease::isDribbleBallPosition()
{
    if(theSpqrDWKcombiner.estimated_ball_relative.x() < 200 &&
            std::fabs(theSpqrDWKcombiner.estimated_ball_relative.y()) < 100)
        return true;
    else return false;

}

bool LibCodeRelease::dribbleAnOpponent()
{
//    for(unsigned int i = 0; i < theSpqrDWKcombiner.opponents.size(); ++i)
//    {
//        if (theSpqrDWKcombiner.opponents.at(i).translation.x() == 0 ||
//                theSpqrDWKcombiner.opponents.at(i).translation.y() == 0)
//            continue;
//        // if an opponent is approaching
//        if( glob2Rel(theSpqrDWKcombiner.opponents.at(i).translation.x(),
//                     theSpqrDWKcombiner.opponents.at(i).translation.y()).translation.x() < 500 &&
//                std::fabs(glob2Rel(theSpqrDWKcombiner.opponents.at(i).translation.x(),
//                                   theSpqrDWKcombiner.opponents.at(i).translation.y()).translation.y()) < 300)
//        {
//            if(theRobotPoseSpqrFiltered.x <= 0.80*SPQR::FIELD_DIMENSION_X &&
//                    std::fabs(angleToGoal) < Angle::fromDegrees(45.f))
//            {
//                if(theSpqrDWKcombiner.estimated_ball_relative.x() <
//                        glob2Rel(theSpqrDWKcombiner.opponents.at(i).translation.x(),
//                                 theSpqrDWKcombiner.opponents.at(i).translation.y()).translation.x()
//                        //                        && isDribbleBallPosition()
//                        )
//                    return true;
//            }
//        }
//    }
    return false;
}

Pose2f LibCodeRelease::getReadyPose(bool kickoff, ContextCoordination::SpqrRole rRole)
{
    if( rRole == ContextCoordination::goalie )
        return glob2Rel(SPQR::GOALIE_BASE_POSITION_X, SPQR::GOALIE_BASE_POSITION_Y);
    else if( rRole == ContextCoordination::striker )
    {
//        if(kickoff) return glob2Rel(SPQR::STRIKER_KICKOFF_POSITION_X, SPQR::STRIKER_KICKOFF_POSITION_Y);
//        else
                return glob2Rel(SPQR::STRIKER_NO_KICKOFF_POSITION_X, SPQR::STRIKER_NO_KICKOFF_POSITION_Y);
    }
    else if( rRole == ContextCoordination::defender )
        return glob2Rel(SPQR::DEFENDER_DEFAULT_POSITION_X, SPQR::DEFENDER_DEFAULT_POSITION_Y);
    else if( rRole == ContextCoordination::supporter )
        return glob2Rel(SPQR::SUPPORTER_DEFAULT_POSITION_X, SPQR::SUPPORTER_DEFAULT_POSITION_Y);
    else if( rRole == ContextCoordination::jolly )
        return glob2Rel(SPQR::JOLLY_DEFAULT_POSITION_X, SPQR::JOLLY_DEFAULT_POSITION_Y);
    else
        return glob2Rel(.0f, SPQR::FIELD_DIMENSION_Y);
}

Pose2f LibCodeRelease::getDefenderPlayingPosition()
{
    return Pose2f( -0.55*SPQR::FIELD_DIMENSION_X, -0.22*SPQR::FIELD_DIMENSION_Y );
}

Pose2f LibCodeRelease::getSupporterPlayingPosition()
{
    return Pose2f( -0.45*SPQR::FIELD_DIMENSION_X, +0.22*SPQR::FIELD_DIMENSION_Y );
}

Pose2f LibCodeRelease::getJollyPlayingPosition()
{
    return Pose2f( 0.25*SPQR::FIELD_DIMENSION_X, -0.33*SPQR::FIELD_DIMENSION_Y );
}

Pose2f LibCodeRelease::getGoalieCoverPosition()
{
	// Decide if to use the global ball or the DWK one
	//~ Pose2f pos;
	//~ if (libCodeRelease.timeSinceBallWasSeen() < 3000) 
		//~ pos = Pose2f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
	//~ else if (theSpqrDWKcombiner.timeSinceWasSeen < 6000)
		//~ pos = Pose2f(theSpqrDWKcombiner.estimated_ball_relative.x(), theSpqrDWKcombiner.estimated_ball_relative.y());
	//~ else 
		//~ pos = Pose2f(8000, 8000); // a big number to have norm outside the range
	//~ float normPos = norm(pos.translation.x(), pos.translation.y()); 
	//~ 
	//~ if (normPos < SPQR::GOALIE_KICK_AWAY_RANGE*3) {
		//~ return Pose2f(SPQR::GOALIE_BASE_POSITION_X, pos.translation.y() / 3000 * 650);
	//~ } else
		return Pose2f(SPQR::GOALIE_BASE_POSITION_X, SPQR::GOALIE_BASE_POSITION_Y);
}

Pose2f LibCodeRelease::rel2Glob(float x, float y)
{
    Vector2f result;
    float rho = sqrt((x * x) + (y * y));

    result.x() = theRobotPoseSpqrFiltered.x + (rho * cos(theRobotPoseSpqrFiltered.theta + atan2(y,x)));
    result.y() = theRobotPoseSpqrFiltered.y + (rho * sin(theRobotPoseSpqrFiltered.theta + atan2(y,x)));

    return Pose2f(result.x(),result.y());
}

Pose2f LibCodeRelease::glob2Rel(float x, float y, float theta)
{
    Vector2f result;

    float tempX = x - theRobotPoseSpqrFiltered.x;
    float tempY = y - theRobotPoseSpqrFiltered.y;

    result.x() = tempX * cos(theRobotPoseSpqrFiltered.theta) + tempY * sin(theRobotPoseSpqrFiltered.theta);
    result.y() = -tempX * sin(theRobotPoseSpqrFiltered.theta) + tempY * cos(theRobotPoseSpqrFiltered.theta);

    return Pose2f(theta /*deg*/, result.x(),result.y());
}

//
Vector2f LibCodeRelease::getTeammatePosition() 
{
	Vector2f obsPos = Vector2f(0,0);
	for (unsigned long i=0; i<theObstacleModel.obstacles.size(); i++) {
		if (theObstacleModel.obstacles.at(i).type == Obstacle::teammate) {
			obsPos = theObstacleModel.obstacles.at(i).center;
			break;
		}
	}
	return obsPos;
}

float LibCodeRelease::getAngleToTeammate() 
{
	return atan2f(getTeammatePosition().y(), getTeammatePosition().x()); 
}
//


/// NoWifiCHallenge functions \\\

bool LibCodeRelease::playLocation() {
	int played = 0;
	int x = theNoWifiPacketToSend.packet.payload.location.x;
	int y = theNoWifiPacketToSend.packet.payload.location.y;
	std::string  sx = std::to_string(x);
	char const *cx = sx.c_str();
	std::cerr << " sizeof " << sx.length() << std::endl;
	int zeros = 4 - sx.length();
	for (int i=0; i< zeros; i++) {
		SystemCall::playSound("audio2600.wav");
		played++;
	}
	for (uint i=0; i<sizeof(cx); i++) {
		switch (cx[i]) {
			case('0'):
				SystemCall::playSound("audio2600.wav");
				played++;
				break;
			case('1'):
				SystemCall::playSound("audio2650.wav");
				played++;
				break;
			case('2'):
				SystemCall::playSound("audio2750.wav");
				played++;
				break;
			case('3'):
				SystemCall::playSound("audio2850.wav");
				played++;
				break;
			case('4'):
				SystemCall::playSound("audio2950.wav");
				played++;
				break;
			case('5'):
				SystemCall::playSound("audio3050.wav");
				played++;
				break;
			case('6'):
				SystemCall::playSound("audio3100.wav");
				played++;
				break;
			case('7'):
				SystemCall::playSound("audio3150.wav");
				played++;
				break;
			case('8'):
				SystemCall::playSound("audio3250.wav");
				played++;
				break;
			case('9'):
				SystemCall::playSound("audio3350.wav");
				played++;
				break;
		}
	}
	std::string  sy = std::to_string(y);
	char const *cy = sy.c_str();
	std::cerr << " sizeof " << sy.length() << std::endl;
	zeros = 4 - sy.length();
	for (int i=0; i< zeros; i++) {
		SystemCall::playSound("audio2600.wav");
		played++;
	}
	for (uint i=0; i<sizeof(cy); i++) {
		switch (cy[i]) {
			case('0'):
				SystemCall::playSound("audio2600.wav");
				played++;
				break;
			case('1'):
				SystemCall::playSound("audio2650.wav");
				played++;
				break;
			case('2'):
				SystemCall::playSound("audio2750.wav");
				played++;
				break;
			case('3'):
				SystemCall::playSound("audio2850.wav");
				played++;
				break;
			case('4'):
				SystemCall::playSound("audio2950.wav");
				played++;
				break;
			case('5'):
				SystemCall::playSound("audio3050.wav");
				played++;
				break;
			case('6'):
				SystemCall::playSound("audio3100.wav");
				played++;
				break;
			case('7'):
				SystemCall::playSound("audio3150.wav");
				played++;
				break;
			case('8'):
				SystemCall::playSound("audio3250.wav");
				played++;
				break;
			case('9'):
				SystemCall::playSound("audio3350.wav");
				played++;
				break;
		}
	}
	SystemCall::playSound("audio3400.wav");
	if (played == 8)
		return true;
	else
		return false;
}


//~ using namespace little_endian_io;
template <typename Word>
ostream& LibCodeRelease::write_word(ostream& outs, Word value, unsigned size)
{
	for (; size; --size, value >>= 8)
		outs.put( static_cast <char> (value & 0xFF) );
	return outs;
}

//METODO PER GENERARE FILE WAV
void LibCodeRelease::generaWav(int prima[70],int seconda[70],string s) {
	ofstream f( s, ios::binary );

	// Write the file headers
	f << "RIFF----WAVEfmt ";     // (chunk size to be filled in later)
	write_word( f,     16, 4 );  // no extension data
	write_word( f,      1, 2 );  // PCM - integer samples
	write_word( f,      2, 2 );  // two channels (stereo file)
	write_word( f,  44100, 4 );  // samples per second (Hz)
	write_word( f, 176400, 4 );  // (Sample Rate * BitsPerSample * Channels) / 8
	write_word( f,      4, 2 );  // data block size (size of two integer samples, one for each channel, in bytes)
	write_word( f,     16, 2 );  // number of bits per sample (use a multiple of 8)

	// Write the data chunk header
	size_t data_chunk_pos = f.tellp();
	f << "data----";  // (chunk size to be filled in later)

	// Write the audio samples
	// (We'll generate a single C4 note with a sine wave, fading from left to right)
	double two_pi = 6.283185307179586476925286766559;
	double max_amplitude = 32760;  // "volume"
	//double max_amplitude = 100000;  // "volume"

	double hz        = 44100;    // samples per second
	double frequency = 3500;  // middle C


	double seconds   = 0.20;      // time
	double seconds3500  = 1.00;      // time

	int N = hz * seconds;  // total number of samples
	int N3500 = hz * seconds3500;  // total number of samples
	for (int n = 0; n < N3500; n++)
	{
		//double amplitude = (double)n / N * max_amplitude;
		double amplitude = 32760;
		//value /=3;
		double value     = sin( (two_pi * n * frequency) / hz );
		write_word( f, (int)(                 amplitude  * value), 2 );
		write_word( f, (int)((                amplitude) * value), 2 );
		//write_word( f, (int)((max_amplitude - amplitude) * value), 2 );
	}
	for(int i=0;i<70;i++)
	{
		for (int n = 0; n < N; n++)
		{
			//double amplitude = (double)n / N * max_amplitude;
			double amplitude = 32760;
			double frequency1 = prima[i];  // middle C
			double frequency2 = seconda[i];  // middle C
			//double value     = (sin( (two_pi * n * frequency1) / hz )+sin( (two_pi * n * frequency2) / hz )+sin( (two_pi * n * frequency3) / hz ))/3;
			double value     = (sin( (two_pi * n * frequency1) / hz )+sin( (two_pi * n * frequency2) / hz ))/2;

			//value /=3;
			//double value     = sin( (two_pi * n * frequency) / hz );
			write_word( f, (int)(                 amplitude  * value), 2 );
			write_word( f, (int)((                amplitude) * value), 2 );
			//write_word( f, (int)((max_amplitude - amplitude) * value), 2 );
		}
	}

	// (We'll need the final file size to fix the chunk sizes above)
	size_t file_length = f.tellp();

	// Fix the data chunk header to contain the data size
	f.seekp( data_chunk_pos + 4 );
	write_word( f, file_length - data_chunk_pos + 8 );

	// Fix the file header to contain the proper RIFF chunk size, which is (file size - 8) bytes
	f.seekp( 0 + 4 );
	write_word( f, file_length - 8, 4 );
}

	
bool LibCodeRelease::playData() {  //TODO testare
	int listLowFreq[16]={1024,1088,1152,1216,1280,1344,1408,1472,1536,1600,1664,1728,1792,1856,1920,1984,};
	int listHighFreq[16]={2560,2624,2688,2752,2816,2880,2944,3008,3072,3136,3200,3264,3328,3392,3456,3520,};
	
	int lowfreq[70], highfreq[70]; 
	for (int i=0; i<70; i++) {
		uint8_t dat = theNoWifiPacketToSend.packet.payload.data.data[i];
		uint8_t low = (dat >> 4);
		uint8_t high = dat & 15;
		lowfreq[i] = listLowFreq[low];
		highfreq[i] = listHighFreq[high];
	}
	
	generaWav(lowfreq, highfreq, "message.wav");
	
	SystemCall::playSound("message.wav");
	
	return false;
}


bool LibCodeRelease::isCloserToBall(){  //TODO CHECK !!!
    float sumR, sumIO, sumB;
    Pose2f a, b;

    sumIO=sqrt(theRobotPoseSpqrFiltered.x*theRobotPoseSpqrFiltered.x + theRobotPoseSpqrFiltered.y*theRobotPoseSpqrFiltered.y);
    sumB=sqrt(theSpqrDWKcombiner.estimated_ball_global.x()*theSpqrDWKcombiner.estimated_ball_global.x()+theSpqrDWKcombiner.estimated_ball_global.y()*theSpqrDWKcombiner.estimated_ball_global.y());

    for(unsigned int j=0; j<theSpqrDWKcombiner.robots_poses.size(); ++j){
        sumR = theSpqrDWKcombiner.robots_poses.at(j).translation.norm();
        //sumR =a.translation.x() + a.translation.y();

        if(sumB-sumR<sumB-sumIO){
            for(unsigned int i=0; i<theSpqrDWKcombiner.opponents.size(); ++i){
                if(theSpqrDWKcombiner.opponents.at(i).estimated_pose.translation.y() != theSpqrDWKcombiner.robots_poses.at(j).translation.y() &&
                        theSpqrDWKcombiner.opponents.at(i).estimated_pose.translation.x() != theSpqrDWKcombiner.robots_poses.at(j).translation.x())
                    return 0;
            }

        }
    }
    return 1;
}

}



