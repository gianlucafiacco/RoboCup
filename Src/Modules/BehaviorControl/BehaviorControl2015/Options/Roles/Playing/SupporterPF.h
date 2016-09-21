
option(SupporterPF)
{

    common_transition
    {
#ifdef AUDIO_IN_BEHAVIORS
        if (option_time < 10)
        {
            std::string wavName = Global::getSettings().robotName.c_str();
            wavName.append(".wav");
            SystemCall::playSound(wavName.c_str());
            SystemCall::playSound("Supporter.wav");
        }
#endif
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
			Pose2f globBall = libCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
			if(theBallModel.estimate.position.x() != 0.0 
					&& theBallModel.estimate.position.x() != 0.0 
					&& theBallModel.estimate.position.norm()<2000
					&& globBall.translation.x() > -3880.f
					&& globBall.translation.x() < -0.2*SPQR::FIELD_DIMENSION_X + 20.f
					&& globBall.translation.y() > 0.f - 20.f
					&& globBall.translation.y() < SPQR::FIELD_DIMENSION_Y)
				goto kickAway;
            if( libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.x, libCodeRelease.getSupporterPlayingPosition().translation.x(), 400) &&
                    libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.y, libCodeRelease.getSupporterPlayingPosition().translation.y(), 400) )
            {
//                if( libCodeRelease.norm(theSpqrDWKcombiner.vel_avg.x(), theSpqrDWKcombiner.vel_avg.y()) < SPQR::MIN_VELOCITY_THRESHOLD )
                    goto turnToOpponentGoal;
            }

        }
        action
        {
            if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(70.f))
                theHeadControlMode = HeadControl::lookAtBall;
            else
                theHeadControlMode = HeadControl::lookLeftAndRight/*UpAndDown*/;

            if( std::abs( libCodeRelease.angleToTarget(
                              libCodeRelease.getSupporterPlayingPosition().translation.x(),
                              libCodeRelease.getSupporterPlayingPosition().translation.y()) ) > Angle::fromDegrees(5.f))
                WalkToTarget(Pose2f(50.f, 0.f, 0.f),
                             Pose2f(libCodeRelease.angleToTarget( libCodeRelease.getSupporterPlayingPosition().translation.x(),
                                                                  libCodeRelease.getSupporterPlayingPosition().translation.y()), 0.f, 0.f));
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
			//~ if(std::abs(theDiveHandle.ballProjectionEstimate) < SPQR::GOALIE_FAR_LIMIT_Y)
			//~ {
				//~ if(theDiveHandle.diveTime < SPQR::GOALIE_DIVE_TIME_TOLERANCE )
				//~ {
					//~ goto stopBall;
				//~ }
			//~ }
					Pose2f globBall = libCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
        if(theBallModel.estimate.position.x() != 0.0 
				&& theBallModel.estimate.position.x() != 0.0 
				&& theBallModel.estimate.position.norm()<2000
				&& globBall.translation.x() > -3880.f
				&& globBall.translation.x() < -0.2*SPQR::FIELD_DIMENSION_X + 20.f
				&& globBall.translation.y() > 0.f - 20.f
				&& globBall.translation.y() < SPQR::FIELD_DIMENSION_Y)
            goto kickAway;
            if(libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getSupporterPlayingPosition().translation.x(),
                                        theRobotPoseSpqrFiltered.y - libCodeRelease.getSupporterPlayingPosition().translation.y()) > 1000)
                goto walkToDynamicPose;

            if(std::abs(libCodeRelease.angleToGoal) > Angle::fromDegrees(15.f))
                goto turnToOpponentGoal;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight/*UpAndDown*/;
            Stand();
        }
    }

    state(turnToOpponentGoal)
    {
        transition
        {
					Pose2f globBall = libCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
        if(theBallModel.estimate.position.x() != 0.0 
				&& theBallModel.estimate.position.x() != 0.0 
				&& theBallModel.estimate.position.norm()<2000
				&& globBall.translation.x() > -3880.f
				&& globBall.translation.x() < -0.2*SPQR::FIELD_DIMENSION_X + 20.f
				&& globBall.translation.y() > 0.f - 20.f
				&& globBall.translation.y() < SPQR::FIELD_DIMENSION_Y)
            goto kickAway;
            if( libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getSupporterPlayingPosition().translation.x(),
                                        theRobotPoseSpqrFiltered.y - libCodeRelease.getSupporterPlayingPosition().translation.y()) > 800)
                goto walkToDynamicPose;

            else if( std::abs(libCodeRelease.angleToGoal) < Angle::fromDegrees(10.f) )
                goto beStationary;

        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight/*UpAndDown*/;
            WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(libCodeRelease.angleToGoal, 0.f, 0.f));
        }
    }

    state(kickAway)
    {
		transition
		{
			std::cerr << "as" << theBallModel.estimate.position.norm() << std::endl;
			if(theBallModel.estimate.position.norm() < 300.f)
				goto alignToGoal;
			else if(theBallModel.estimate.position.norm() > 300.f)
				goto turnToBall;
			else goto beStationary;
			
		}
		action
		{
			if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                theHeadControlMode = HeadControl::lookLeftAndRight;
            else
                theHeadControlMode = HeadControl::lookAtBall;
			Stand();
		}
		
        //~ transition
        //~ {
            //~ if( theBallModel.estimate.position.norm() > 800  || action_done || libCodeRelease.timeSinceBallWasSeen() > 1000|| state_time > 10000)
                //~ goto beStationary;
        //~ }
        //~ action
        //~ {
            //~ if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                //~ theHeadControlMode = HeadControl::lookLeftAndRight;
            //~ else
                //~ theHeadControlMode = HeadControl::lookAtBall;
//~ 
            //~ KickAway();
        //~ }
    }
    
    state(turnToBall)
  {
	  transition
	  {
          if(theBallModel.estimate.position.norm() < 300.f)
			  goto alignToGoal;

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
        if(theBallModel.estimate.position.norm() < 300.f)
			goto alignToGoal;
	}
	action
    {
        WalkToTarget(Pose2f(.2f, SPQR::WALKING_VELOCITY_X, 20.f), theBallModel.estimate.position);
	}
  }

  state(alignToGoal)
  {
	  transition
	  {
          if(std::abs(libCodeRelease.angleToGoal) < Angle::fromDegrees(10.f) && std::abs(theBallModel.estimate.position.y()) < 100.f)
			  goto alignBehindBall;
	  }
	  action
      {
          WalkToTarget(Pose2f(0.4f, 10.f, 30.f), Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 300.f, theBallModel.estimate.position.y()));
	  }
  }

  state(alignBehindBall)
  {
	  transition //TODO francesco: check conditions 
	  {

          if(libCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 60.f) ||
                  libCodeRelease.between(theBallModel.estimate.position.y(), -60.f, -30.f) )
		  {
              if( libCodeRelease.between(theBallModel.estimate.position.x(), 100.f, 170.f) &&
                      std::abs(libCodeRelease.angleToGoal) < Angle::fromDegrees(3.f) )
				  goto kick;
		  }
	  }
	  action
      {
          if( theBallModel.estimate.position.y() >= .0f )
              WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                           Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 50.f));
		  else
              WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                           Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() + 50.f));
	  }
  }


  state(kick)
  {
	  transition
	  {
		  if (action_done || state_time > 3000)
			goto beStationary;
	  }
	  action
	  {
		  theHeadControlMode = HeadControl::lookAtBall;
          Kicks("fastForwardKick");
	  }
  }
        
	state(stopBall)
    {
        transition
        {
            if (state_time > SPQR::GOALIE_DIVE_TIME)
				goto beStationary;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            StopBall();
        }
    }
}

