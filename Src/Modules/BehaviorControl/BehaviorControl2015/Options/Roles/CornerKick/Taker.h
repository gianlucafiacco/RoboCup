/*
 * corner taker 
 * raggiunge la palla muove la testa a destra e sinistra, e fa il kick
 * stati: walkToBall, AlignToGoal e Kick oppure PassToTarget al posto del kick
*/

//#define SPQR_DEBUG_TAKER

option(Taker)
{

    initial_state(start)
    {
        transition
        {
#ifdef SPQR_DEBUG_TAKER
            STATE("start");
#endif
            if(libCodeRelease.timeSinceBallWasSeen() < 1000 && theBallModel.estimate.position.norm() < 2500.f)
                goto walkToBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            Stand();
        }
    }

//    state(searchForBall)
//    {
//        transition
//        {
//#ifdef SPQR_DEBUG_TAKER
//            STATE("searchForBall");
//#endif
//            if(libCodeRelease.timeSinceBallWasSeen() < 300)
//                goto turnToBall;
//        }
//        action
//        {
//            theHeadControlMode = HeadControl::lookForward;

//            if( libCodeRelease.ballOutOnLeft )
//                WalkAtSpeedPercentage(Pose2D(-.8f, 0.f, 0.f));
//            else
//                WalkAtSpeedPercentage(Pose2D(.8f, 0.f, 0.f));
//        }
//    }



    state(turnToBall)
    {
        transition
        {
#ifdef SPQR_DEBUG_TAKER
            STATE("turnToBall");
#endif
//            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
//                goto searchForBall;
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
#ifdef SPQR_DEBUG_TAKER
            STATE("walkToBall");
#endif
            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(5.f) || theBallModel.estimate.position.x() < .0f)
                goto turnToBall;

            if( theBallModel.estimate.position.norm() < 300.f )
                goto kick;

//            if( libCodeRelease.timeSinceBallWasSeen() > 1000 )
//                goto searchForBall;
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTarget( Pose2f(.2f, 70.f, 30.f), Pose2f(theBallModel.estimate.position - Vector2f(200.0f, .0f) ));
        }
    }


    state(kick)
    {
        transition
        {
#ifdef SPQR_DEBUG_TAKER
            STATE("kick");
#endif
            if(  (state_time > 10000 && action_done) )
                goto start;
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            //Kicks("forwardKick");
            KickToTarget(Pose2f(/*BALL destination computed*/theFieldDimensions.xPosOpponentGroundline, 0.0));
        }
    }

}
