/*
 * corner kicker
*/

//#define SPQR_DEBUG_KICKER

option(Kicker)
{

    initial_state(start)
    {
        transition
        {
#ifdef SPQR_DEBUG_KICKER
            STATE("start");
#endif
            goto walkToBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            Stand();
        }
    }

    state(turnToBall)
    {
        transition
        {
#ifdef SPQR_DEBUG_KICKER
            STATE("turnToBall");
#endif
            if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(5.f))
                goto walkToBall;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
        }
    }

    state(walkToBall)
    {
        transition
        {
#ifdef SPQR_DEBUG_KICKER
            STATE("walkToBall");
#endif
            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(5.f) || theBallModel.estimate.position.x() < .0f)
                goto turnToBall;

            if( theBallModel.estimate.position.norm() < 300.f && libCodeRelease.timeSinceBallWasSeen() < 300)
                goto kick;
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTarget( Pose2f(.2f, 70.f, 30.f), Pose2f(theBallModel.estimate.position - Vector2f(200.0f, .0f)) );
        }
    }


    state(kick)
    {
        transition
        {
#ifdef SPQR_DEBUG_KICKER
            STATE("kick");
#endif
            if (state_time > 30000){
                goto Stand_state;
            }
        }
        action
        {

            //theHeadControlMode = HeadControl::lookLeftAndRight;
            theHeadControlMode = HeadControl::lookAtBall;

            Pose2f robotPose = Pose2f(theRobotPoseSpqrFiltered.theta, theRobotPoseSpqrFiltered.x,theRobotPoseSpqrFiltered.y);
            float kickAngle = 0.f;//AngleForKickBetweenObstacleProvider::angleProvider(robotPose, theSpqrDWKcombiner.opponents_estimated_poses);
#ifdef DEBUG_ANGLE_FOR_KICK
            cout<<"kickAngle = "<< 57.f * kickAngle <<endl;
#endif
            KickToTarget(Pose2f( theFieldDimensions.xPosOpponentGroundline - 2000.f * std::sin(kickAngle),
                                 theFieldDimensions.yPosLeftSideline - 2000.f * std::cos(kickAngle)));
        }
    }
    state(Stand_state){
        action{
            Stand();
        }
    }

}
