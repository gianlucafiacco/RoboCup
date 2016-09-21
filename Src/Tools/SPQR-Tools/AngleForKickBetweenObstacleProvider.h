#pragma once

#include "Tools/Math/Pose2f.h"

#include <map>

#define ANGLE_MIN_INTERVAL 5.f  //degrees
#define OBSTACLE_WIDTH 350.f    //mm
#define MIN_ANGULAR_WINDOWS_TO_PERFORM_KICKS 10.f //degrees
#define STARTING_ANGLES 0.f     //degrees of the starting angular value
#define ZERO_OPPONENT_ANGLE 37.f


struct ObstacleAngular
{
    float thetaAzimuth = 0;
    float thetaWidth = 0;
};

class AngleForKickBetweenObstacleProvider       //TODO Vincenzo: Using the ObstacleModel to model the Robot and chose the angle among them
{
public:
    //NB: this works for obstacle percept in relative coordinates
    static float angleProvider (/*std::vector<Pose2f> opponents_estimated_poses*/ Pose2f robotPose, std::vector<Pose2f> opponents_estimated_poses)
    {
        struct ObstacleAngular ObstacleAngularArr[5];

        bool not_opponents = true;
#ifdef DEBUG_ANGLE_FOR_KICK
        cout<< "angular values are in degrees in the debug prints!"<<endl;
#endif
        for( uint i = 0 ; i < opponents_estimated_poses.size(); i++ )
        {
            if(opponents_estimated_poses.at(i).translation.x() != 0 &&  opponents_estimated_poses.at(i).translation.y() != 0)
            {
                not_opponents = false;

                Pose2f iThObstacleInRelative = glob2RelFromPose2f(robotPose,opponents_estimated_poses.at(i));

                ObstacleAngularArr[i].thetaWidth = std::atan2(OBSTACLE_WIDTH, norm( iThObstacleInRelative.translation.x(),
                                                                                    iThObstacleInRelative.translation.y()) ) ;
                ObstacleAngularArr[i].thetaAzimuth = std::atan2( iThObstacleInRelative.translation.y(),
                                                                 iThObstacleInRelative.translation.x());
#ifdef DEBUG_ANGLE_FOR_KICK
                cout<<"obstacle number = "<< i << ", position: "<< opponents_estimated_poses.at(i).translation.x << ", " <<
                      opponents_estimated_poses.at(i).translation.y << " thetaAzimuth = " << 57 * ObstacleAngularArr[i].thetaAzimuth <<
                      " thetaWidth ="<< 57 * ObstacleAngularArr[i].thetaWidth<<endl;
#endif
            }
        }

        //ObstacleAngularArr =  AngularSorter(ObstacleAngularArr, opponents_estimated_poses); //sort the vector of struct for the Angular values

        float tmpMaximumDistance = Angle::fromDegrees(MIN_ANGULAR_WINDOWS_TO_PERFORM_KICKS);
        uint lowIndexOfMaxDistance = 0;
        for (uint i = 0; i < (opponents_estimated_poses.size() - 1) ; i++ )
        {
            if (ObstacleAngularArr[i+1].thetaAzimuth - ObstacleAngularArr[i].thetaAzimuth
                    - 0.5 * (ObstacleAngularArr[i].thetaWidth + ObstacleAngularArr[i+1].thetaWidth) > tmpMaximumDistance)
            {
                tmpMaximumDistance = ObstacleAngularArr[i+1].thetaAzimuth - ObstacleAngularArr[i].thetaAzimuth
                        - 0.5 * (ObstacleAngularArr[i].thetaWidth + ObstacleAngularArr[i+1].thetaWidth);
                lowIndexOfMaxDistance = i;
            }

        }

        if(not_opponents)
            return Angle::fromDegrees(ZERO_OPPONENT_ANGLE);
        else
        {
            if ( tmpMaximumDistance == Angle::fromDegrees(MIN_ANGULAR_WINDOWS_TO_PERFORM_KICKS) ) //no free space between Obstacles case
            {
#ifdef DEBUG_ANGLE_FOR_KICK
                cout<<"no free space between Obstacles case"<<endl;
#endif
                if ((ObstacleAngularArr[0].thetaAzimuth) - Angle::fromDegrees(STARTING_ANGLES) > Angle::fromDegrees(MIN_ANGULAR_WINDOWS_TO_PERFORM_KICKS ))
                {
                    //there is enough space from starting point to the first obstacle
                    return ((ObstacleAngularArr[0].thetaAzimuth) - Angle::fromDegrees(MIN_ANGULAR_WINDOWS_TO_PERFORM_KICKS ));
                }
                else //there is no space neither between obstacles and before the starting angle so the bast angle is after the last obstacle
                {
                    return ((ObstacleAngularArr[opponents_estimated_poses.size()-1].thetaAzimuth)
                            + 0.5 * (ObstacleAngularArr[opponents_estimated_poses.size()-1].thetaAzimuth) + Angle::fromDegrees(MIN_ANGULAR_WINDOWS_TO_PERFORM_KICKS ));
                }
            }

            return (ObstacleAngularArr[lowIndexOfMaxDistance + 1].thetaAzimuth - ObstacleAngularArr[lowIndexOfMaxDistance].thetaAzimuth);
        }
    }

    std::vector<ObstacleAngular> AngularSorter (std::vector<ObstacleAngular> unsortedVec, std::vector<Pose2f> opponents_estimated_poses){
        //std::map <float key, ObstacleAngular, Compare = less<ObstacleAngular.thetaAzimuth>> value_type;
        //typedef pair <const Key, ObstacleAngular.thetaAzimuth> value_type;

        //        float currentMinAngle = 0.0f;
        //        struct ObstacleAngular TmpVect[5];
        //        for (uint i = 0 ; i < opponents_estimated_poses.size(); i++) {
        //            //ObstacleAngularArr.pop();
        //            for (uint j = 0 ; j < opponents_estimated_poses.size(); j++) {
        //                if (TmpVect[i] < TmpVect[j] && TmpVect[j] > currentMinAngle)
        //                    ;
        //            }
        //        }

        //sorting TODO
        return unsortedVec;
    }

    static float norm(float x, float y)
    {
        return sqrt((x*x) + (y*y));
    }

    static Pose2f glob2RelFromPose2f(Pose2f obstaclePose, Pose2f robotPose)
    {
        Vector2f result;

        float tempX = obstaclePose.translation.x() - robotPose.translation.x();
        float tempY = obstaclePose.translation.y() - robotPose.translation.y();

        result.x() = tempX * cos(robotPose.rotation) + tempY * sin(robotPose.rotation);
        result.y() = -tempX * sin(robotPose.rotation) + tempY * cos(robotPose.rotation);

        return Pose2f(obstaclePose.rotation /*deg*/, result.x(),result.y());
    }
};
