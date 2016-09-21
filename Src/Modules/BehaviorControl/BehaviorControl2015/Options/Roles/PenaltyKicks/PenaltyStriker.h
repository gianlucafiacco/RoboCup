

option(PenaltyStriker)
{

    common_transition
    {


        if ((theKeyStates.pressed[KeyStates::headRear]))
            goto start;
    }

    initial_state(start)
    {
        transition
        {
            // no kick-off & the other team touches the ball or 10 seconds
//            if(theGameInfo.kickOffTeam != theOwnTeamInfo.teamColor)
//            {
//                if((libCodeRelease.ballIsInGame() || state_time > 5000))
//                    goto turnToBall;
//            }
//            // kick-off
//            else if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamColor)
//                goto turnToBall;

            if ((theKeyStates.pressed[KeyStates::headFront]))
                goto dead;


            if ((theKeyStates.pressed[KeyStates::headMiddle]))
            {
                std::string wavName = Global::getSettings().robotName.c_str();
                wavName.append(".wav");
                SystemCall::playSound(wavName.c_str());
                SystemCall::playSound("Striker.wav");
                goto /*alignBehindBall*/ kick;
            }




        }
        action
        {
            cout << (theKeyStates.pressed[KeyStates::headFront]) << endl;

            theHeadControlMode = HeadControl::lookAtBall;
            Stand();
        }
    }

    state(searchForBall)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() < 300)
                goto turnToBall;
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



    state(turnToBall)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto searchForBall;
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
            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(5.f) || theBallModel.estimate.position.x() < .0f)
                goto turnToBall;

            if( theBallModel.estimate.position.norm() < 300.f )
                goto alignToGoal;

            if( libCodeRelease.timeSinceBallWasSeen() > 1000 )
                goto searchForBall;
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTarget( Pose2f(.2f, 70.f, 30.f), Pose2f(theBallModel.estimate.position - Vector2f(200.0f, .0f)) );
        }
    }

    state(alignToGoal)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto searchForBall;


            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(50.f) && theBallModel.estimate.position.x() < 50.0f)
                goto turnToBall;

            if(std::abs(libCodeRelease.penaltyAngle) < Angle::fromDegrees(5.f) && std::abs(theBallModel.estimate.position.y()) < 100.f)
                goto alignBehindBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTarget(Pose2f(0.4f, .3f, .7f),
                         Pose2f(libCodeRelease.penaltyAngle, theBallModel.estimate.position.x() - 250.f, theBallModel.estimate.position.y()));
        }
    }


    state(alignBehindBall)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto searchForBall;

            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(50.f) && theBallModel.estimate.position.x() < 50.0f)
                goto turnToBall;

            // y condition
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 90.f) ||
                    libCodeRelease.between(theBallModel.estimate.position.y(), -90.f, -30.f) )
            {
                // x condition
                if( libCodeRelease.between(theBallModel.estimate.position.x(), 80.f, 165.f) &&
                        std::abs(libCodeRelease.penaltyAngle) < Angle::fromDegrees(5.f))
                    goto kick;
            }

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            if( theBallModel.estimate.position.y() >= .0f )
                WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                             Pose2f(libCodeRelease.penaltyAngle,
                                    theBallModel.estimate.position.x() - 155.f,
                                    theBallModel.estimate.position.y() - 90.f));
            else
                WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                             Pose2f(libCodeRelease.penaltyAngle,
                                    theBallModel.estimate.position.x() - 155.f,
                                    theBallModel.estimate.position.y() + 90.f));
        }
    }

    state(kick)
    {
        transition
        {
//            if( state_time > 3000 || (state_time > 10 && action_done) )
//                goto stop;

            if ((theKeyStates.pressed[KeyStates::headRear]))
                goto start;
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            Kicks("realBallKickWithStand");
        }
    }

    state(stop)
    {
        transition
        {
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            Stand();
        }
    }

    state(dead)
    {
        transition
        {

            if ((theKeyStates.pressed[KeyStates::headMiddle]))
                goto alignBehindBall;

        }
        action
        {
            SpecialAction(SpecialActionRequest::playDead);
        }
    }

}
