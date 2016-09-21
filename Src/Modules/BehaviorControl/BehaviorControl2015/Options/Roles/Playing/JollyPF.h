option(JollyPF)
{
    common_transition
    {
#ifdef AUDIO_IN_BEHAVIORS
        if (option_time < 10)
        {
            std::string wavName = Global::getSettings().robotName.c_str();
            wavName.append(".wav");
            SystemCall::playSound(wavName.c_str());
            SystemCall::playSound("Jolly.wav");
        }
#endif
        if(theBallModel.estimate.position.norm()<500)
            goto kickAway;
    }

    initial_state(start)
    {
        transition
        {
            goto walkToDynamicPose;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            Stand();
        }
    }

    state(walkToDynamicPose)
    {
        transition
        {
            if( libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.x,libCodeRelease.getJollyPlayingPosition().translation.x(),400) &&
                    libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.y,libCodeRelease.getJollyPlayingPosition().translation.y(),400) )
            {
                //                if( libCodeRelease.norm(theSpqrDWKcombiner.vel_avg.x, theSpqrDWKcombiner.vel_avg.y) < SPQR::MIN_VELOCITY_THRESHOLD )
                goto turnToBall;
            }

        }
        action
        {
            if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(70.f))
                theHeadControlMode = HeadControl::lookAtBall;
            else
                theHeadControlMode = HeadControl::lookLeftAndRight/*UpAndDown*/;

            if( std::abs( libCodeRelease.angleToTarget(
                              libCodeRelease.getJollyPlayingPosition().translation.x(),
                              libCodeRelease.getJollyPlayingPosition().translation.y()) ) > Angle::fromDegrees(5.f))
                WalkToTarget(Pose2f(50.f, 0.f, 0.f),
                             Pose2f(libCodeRelease.angleToTarget( libCodeRelease.getJollyPlayingPosition().translation.x(),
                                                                  libCodeRelease.getJollyPlayingPosition().translation.y()), 0.f, 0.f));
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
            if(libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getJollyPlayingPosition().translation.x(),
                                   theRobotPoseSpqrFiltered.y - libCodeRelease.getJollyPlayingPosition().translation.y()) > 1000)
                goto walkToDynamicPose;

            if(std::abs(libCodeRelease.angleToGoal) > Angle::fromDegrees(15.f))
                goto turnToBall;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight/*UpAndDown*/;
            Stand();
        }
    }

    state(turnToBall)
    {
        transition
        {
            if( libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getJollyPlayingPosition().translation.x(),
                                    theRobotPoseSpqrFiltered.y - libCodeRelease.getJollyPlayingPosition().translation.y()) > 800)
                goto walkToDynamicPose;

            else if( std::abs(libCodeRelease.angleToTarget(theSpqrDWKcombiner.estimated_ball_global.x(),
                                                           theSpqrDWKcombiner.estimated_ball_global.y())) < Angle::fromDegrees(10.f) )
                goto beStationary;


        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight/*UpAndDown*/;
            WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(libCodeRelease.angleToTarget(theSpqrDWKcombiner.estimated_ball_global.x(),
                                                                                     theSpqrDWKcombiner.estimated_ball_global.y()), 0.f, 0.f));
        }
    }

    state(kickAway)
    {
        transition
        {
            if( (state_time > 10000 && action_done) || (theBallModel.estimate.position.norm() > 800) )
                goto beStationary;
        }
        action
        {
            if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                theHeadControlMode = HeadControl::lookLeftAndRight;
            else
                theHeadControlMode = HeadControl::lookAtBall;

            GoalieKickAway();
        }
    }
}
