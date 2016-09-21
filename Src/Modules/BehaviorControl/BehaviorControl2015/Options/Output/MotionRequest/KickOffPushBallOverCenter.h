
//#define SPQR_KICKOFF_OVER_CIRCLE

option(KickOffPushBallOverCenter)
{
    initial_state(start)
    {
        transition
        {
#ifdef SPQR_KICKOFF_OVER_CIRCLE
            STATE("start");
#endif
            if(libCodeRelease.timeSinceBallWasSeen() < 300) goto walkToBall;

            else goto lookAround;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            Stand();
        }
    }

    state(lookAround)
    {
        transition
        {
#ifdef SPQR_KICKOFF_OVER_CIRCLE
            STATE("lookAround");
#endif
            if(libCodeRelease.timeSinceBallWasSeen() < 300)
                goto walkToBall;
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

    state(walkToBall)
    {
        transition
        {
#ifdef SPQR_KICKOFF_OVER_CIRCLE
            STATE("walkToBall");
#endif
            if( theBallModel.estimate.position.norm() < 300.f )
                goto alignToGoal;

            if( libCodeRelease.timeSinceBallWasSeen() > 3000 )
                goto lookAround;
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;

            if(std::abs(theBallModel.estimate.position.angle()) > Angle::Angle::fromDegrees(5.f))
                WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
            else
                WalkToTarget( Pose2f(.2f, SPQR::WALKING_VELOCITY_X, 30.f), Pose2f(theBallModel.estimate.position - Vector2f(200.0f, .0f)) );

        }
    }

    state(alignToGoal)
    {
        transition
        {
#ifdef SPQR_KICKOFF_OVER_CIRCLE
            STATE("alignToGoal");
#endif
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto lookAround;

            if( std::abs(theBallModel.estimate.position.angle()) > Angle::Angle::fromDegrees(50.f) ||
                    theBallModel.estimate.position.x() < 100.0f ||
                    std::fabs(theBallModel.estimate.position.y()) > 250.0f )
                goto walkToBall;

            if(std::fabs(libCodeRelease.angleToGoal) < libCodeRelease.kickAngle &&
                    std::fabs(theRobotPoseSpqrFiltered.theta)<Angle::fromDegrees(45) &&
                    std::fabs(theBallModel.estimate.position.y()) < 150.f)
                goto alignBehindBall;

            if(state_time > 20000) goto kick;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTarget(Pose2f(0.4f, .3f, .7f),
                         Pose2f(libCodeRelease.angleToGoal,
                                theBallModel.estimate.position.x() - SPQR::ALIGN_TO_GOAL_X,
                                theBallModel.estimate.position.y() ));
        }
    }

    state(alignBehindBall)
    {
        transition
        {
#ifdef SPQR_KICKOFF_OVER_CIRCLE
            STATE("alignBehindBall");
#endif
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto lookAround;

            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(50.f) /*&& theBallModel.estimate.position.x < 50.0f*/
                    || std::abs(theBallModel.estimate.position.y())> 250.f)
                goto walkToBall;


            // y condition
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 0.f, 50.f) ||  //30,50
                    libCodeRelease.between(theBallModel.estimate.position.y(), -50.f, -0.f) ) //-50, -30
            {
                // x condition
                if( libCodeRelease.between(theBallModel.estimate.position.x(), 30.f, 170.f) &&
                        std::abs(libCodeRelease.angleToGoal) < libCodeRelease.kickAngle )
                {

//                     if( libCodeRelease.dribbleAnOpponent() ) goto alignDribble;
// 
                    // dribble ball condition
                    if( theSpqrDWKcombiner.estimated_ball_global.norm()  < 900.f)
                        goto dribbleBallAhead;
// 
// #ifndef NOT_KICKING_STRIKER
                    // kick condition
                        goto kick;
//#endif
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
#ifdef SPQR_KICKOFF_OVER_CIRCLE
            STATE("dribbleBallAhead");
#endif

            if( libCodeRelease.timeSinceBallWasSeen() > 3000 )
                goto lookAround;

            if( theBallModel.estimate.position.norm() > 300.f )
                goto walkToBall;

            if( theSpqrDWKcombiner.estimated_ball_global.norm() > 950.f)
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
#ifdef SPQR_KICKOFF_OVER_CIRCLE
            STATE("kick");
#endif
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
}
