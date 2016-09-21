#include <iostream>
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"

option(DropIn)
{
    common_transition{
           for(auto teammate : theTeammateData.teammates){
                if(teammate.standardBehaviorStatus.intention == DROPIN_INTENTION_KICK){
                    theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_DEFENSIVE;
                    goto defense;
                }
           }
    }

    initial_state(start)
    {
        transition
        {
            goto lookAround;
        }

        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            Stand();
        }
    }


    state(lookAround){
        transition
        {

            if(theBallModel.estimate.position.norm()<2000.f && libCodeRelease.isCloserToBall())
                goto attack;
            else
            goto defense;

        }

        action
        {
            theHeadControlMode = HeadControl::lookForward;
            if( libCodeRelease.ballOutOnLeft )
                WalkAtSpeedPercentage(Pose2f(-.8f, 0.f, 0.f));
            else
                WalkAtSpeedPercentage(Pose2f(.8f, 0.f, 0.f));
        }
    }
    state(attack){
        transition{

            if(theBallModel.estimate.position.norm()>2000.f || !libCodeRelease.isCloserToBall())
                goto defense;

            else goto walkToBall;
        }
        action{
            theHeadControlMode = HeadControl::lookForward;
            Stand();
        }
    }





    state(walkToBall)
    {
        transition
        {
            theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_KICK;
            if( theBallModel.estimate.position.norm() > 2000.f || !libCodeRelease.isCloserToBall())
                goto defense;

            if( theBallModel.estimate.position.norm() < 400.f )
                goto alignToGoal;

            if( libCodeRelease.timeSinceBallWasSeen() > 3000 )
                goto lookAround;
        }
        action
        {
            if( theSpqrDWKcombiner.estimated_ball_global.norm() > 1000.f || theBallModel.estimate.position.norm() > 1000.f)
            {
                if( theSpqrDWKcombiner.estimated_ball_global.norm() < 2500.f || theBallModel.estimate.position.norm() < 2500.f)
                    theHeadControlMode = HeadControl::lookLeftAndRightForStriker;
                else
                    theHeadControlMode = HeadControl::lookAtLandmark;
            }
            else
            {
                theHeadControlMode = HeadControl::lookAtGlobalBall;
            }

            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(10.f))
                WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
            else
            {
                if(theBallModel.estimate.position.norm() > 500.f)
                    WalkToTarget( Pose2f(.0f, SPQR::WALKING_VELOCITY_X, 0.f), Pose2f(theBallModel.estimate.position - Vector2f(200.0f, .0f)) );
                else
                    WalkToTarget( Pose2f(.0f, SPQR::WALKING_VELOCITY_X*0.8f, 0.f), Pose2f(theBallModel.estimate.position - Vector2f(200.0f, .0f)) );
            }

        }
    }

    state(alignToGoal)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto lookAround;

            if( std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(50.f) ||
                    theBallModel.estimate.position.x() < 100.0f ||
                    std::fabs(theBallModel.estimate.position.y()) > 250.0f )
                goto walkToBall;

            // dribble ball condition
            if( theRobotPoseSpqrFiltered.x > -0.50f*SPQR::FIELD_DIMENSION_X &&
                    theRobotPoseSpqrFiltered.x <= 0.4f*SPQR::FIELD_DIMENSION_X)
                goto dribbleBallAhead;

            if(std::fabs(libCodeRelease.angleToGoal) < libCodeRelease.kickAngle &&
                    std::fabs(theBallModel.estimate.position.y()) < 150.f)
                goto alignBehindBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTarget(Pose2f(0.4f, .3f, .7f),
                         Pose2f(libCodeRelease.angleToGoal,
                                theBallModel.estimate.position.x() - SPQR::ALIGN_TO_GOAL_X,
                                theBallModel.estimate.position.y()));
        }
    }

    state(alignBehindBall)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto lookAround;

            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(50.f) /*&& theBallModel.estimate.position.x() < 50.0f*/
                    || std::abs(theBallModel.estimate.position.y())> 250.f)
                goto walkToBall;


            // y condition
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 0.f, 90.f) ||  //30,50
                    libCodeRelease.between(theBallModel.estimate.position.y(), -90.f, -0.f) ) //-50, -30
            {
                // x condition
                if( libCodeRelease.between(theBallModel.estimate.position.x(), 30.f, 170.f) &&
                        std::abs(libCodeRelease.angleToGoal) < libCodeRelease.kickAngle )
                {

                    // dribble ball condition
                    if( theRobotPoseSpqrFiltered.x > -0.50f*SPQR::FIELD_DIMENSION_X &&
                            theRobotPoseSpqrFiltered.x <= 0.4f*SPQR::FIELD_DIMENSION_X)
                        goto dribbleBallAhead;

                    goto kick;
                }
            }
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            if( theBallModel.estimate.position.y() >= .0f )
                WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                             Pose2f(libCodeRelease.angleToGoal+libCodeRelease.correctionKickAngle,
                                    theBallModel.estimate.position.x() - SPQR::ALIGN_BEHIND_BALL_X,
                                    theBallModel.estimate.position.y() - SPQR::ALIGN_BEHIND_BALL_Y));
            else
                WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                             Pose2f(libCodeRelease.angleToGoal-libCodeRelease.correctionKickAngle,
                                    theBallModel.estimate.position.x() - SPQR::ALIGN_BEHIND_BALL_X,
                                    theBallModel.estimate.position.y() + SPQR::ALIGN_BEHIND_BALL_Y));
        }
    }

    state(dribbleBallAhead)
    {
        transition
        {

            if( libCodeRelease.timeSinceBallWasSeen() > 3000 )
                goto lookAround;

            if( theBallModel.estimate.position.norm() > 300.f )
                goto walkToBall;

            // kick condition
            if( !(theRobotPoseSpqrFiltered.x > -0.55f*SPQR::FIELD_DIMENSION_X &&
                  theRobotPoseSpqrFiltered.x <= 0.4f*SPQR::FIELD_DIMENSION_X) )
                goto alignBehindBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTarget( Pose2f(.2f, SPQR::WALKING_VELOCITY_X, 30.f), Pose2f(theBallModel.estimate.position + Vector2f(150.0f, .0f)) );
        }
    }



    state(kick)
    {
        transition
        {
            if( state_time > 3000 || (state_time > 10 && action_done) )
                goto lookAround;

            if(theBallModel.estimate.position.norm() > 500.f)
                goto walkToBall;
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
                Kicks("forwardKick");
        }
    }

    state(defense)
    {
        transition{

            theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_DEFENSIVE;

            if(theBallModel.estimate.position.norm()<2000 && libCodeRelease.isCloserToBall()){
                goto attack;
            }
            else goto walkToDynamicPose;
        }
        action{
            theHeadControlMode = HeadControl::lookForward;

        }
    }
    state(walkToDynamicPose)
    {
        transition
        {
            if( libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.x,libCodeRelease.getDefenderPlayingPosition().translation.x(),500) &&
                    libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.y,libCodeRelease.getDefenderPlayingPosition().translation.y(),500) )
            {
                    goto turnToOpponentGoal;
            }
            if(theBallModel.estimate.position.norm()<1000 && libCodeRelease.isCloserToBall()){
                goto attack;
            }

        }
        action
        {
            if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(45.f))
                theHeadControlMode = HeadControl::lookAtBall;
            else
                theHeadControlMode = HeadControl::lookLeftAndRightUpAndDown;


            if( std::abs( libCodeRelease.angleToTarget(
                              libCodeRelease.getDefenderPlayingPosition().translation.x(),
                              libCodeRelease.getDefenderPlayingPosition().translation.y()) ) > Angle::fromDegrees(5.f))
                WalkToTarget(Pose2f(50.f, 0.f, 0.f),
                             Pose2f(libCodeRelease.angleToTarget( libCodeRelease.getDefenderPlayingPosition().translation.x(),
                                                                  libCodeRelease.getDefenderPlayingPosition().translation.y()), 0.f, 0.f));
            else
            {
                if (theSpqrDWKcombiner.vel_avg.x() < 40.f)
                    WalkAtSpeed( Pose2f(0.f, 40.f, theSpqrDWKcombiner.vel_avg.y()) );
                else
                    WalkAtSpeed( Pose2f(0.f, theSpqrDWKcombiner.vel_avg.x(), theSpqrDWKcombiner.vel_avg.y()) );
            }
        }
    }



    state(beStationary)
    {
        transition
        {
            if(theBallModel.estimate.position.norm()<1000 && libCodeRelease.isCloserToBall()){
                goto attack;
            }
            if(libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getDefenderPlayingPosition().translation.x(),
                                   theRobotPoseSpqrFiltered.y - libCodeRelease.getDefenderPlayingPosition().translation.y()) > 800)
            {
                if(std::fabs(theSpqrDWKcombiner.estimated_ball_global.y()) > 500)
                    goto walkToDynamicPose;
            }

            if(std::abs(libCodeRelease.angleToGoal) > Angle::fromDegrees(20.f))
                goto turnToOpponentGoal;

        }
        action
        {
            if(theFrameInfo.time - theBallModel.timeWhenLastSeen < 1000)
                theHeadControlMode = HeadControl::lookAtBall;
            else
                theHeadControlMode = HeadControl::lookLeftAndRightUpAndDown;

            Stand();
        }
    }

    state(turnToOpponentGoal)
    {
        transition
        {

            if( libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getDefenderPlayingPosition().translation.x(),
                                    theRobotPoseSpqrFiltered.y - libCodeRelease.getDefenderPlayingPosition().translation.y()) > 800)
                goto walkToDynamicPose;

            if( libCodeRelease.between(theRobotPoseSpqrFiltered.theta,
                                                   libCodeRelease.angleToGoal + Angle::fromDegrees(-10.f) ,
                                                   libCodeRelease.angleToGoal + Angle::fromDegrees( 10.f)  ) )
                goto beStationary;

        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRightUpAndDown;
            WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(libCodeRelease.angleToGoal, 0.f, 0.f));
        }
    }

}
