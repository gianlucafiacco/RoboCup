
//#define SPQR_DEBUG_GOALIE
 #define SPQR_GOALIE_STAND
 #define SPQR_GOALIE_LOOK_AROUND // ??
// #define SPQR_GOALIE_NOT_BACKTRACK

//~ TODO maybe:  
//~ 	- tiri diversi usare 
//~ 	- usera DWKCombinerBall invece che ballmodel? 

// look at ball


option(Goalie)
{
	common_transition
	{
		
#ifndef SPQR_GOALIE_STAND
 #ifdef SPQR_DEBUG_GOALIE
		std::cerr << "in range : " <<libCodeRelease.isBallInKickAwayRange() << std::endl;
		std::cerr << "seen ball at : " << theBallModel.estimate.position.x() << ", " << theBallModel.estimate.position.y() << std::endl;
 #endif

#endif
	}
	
    initial_state(start)
    {
        transition
        {
#ifdef SPQR_GOALIE_STAND
            goto mainLoop;
#endif

			theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_KEEPER;
            if( !libCodeRelease.isGoalieInStartingPosition() )
                goto gotoGoaliePosition;

            if(state_time > 500)
                goto turnToOpponentGoal;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForwardGoalie;
            Stand();
        }
    }

    state(gotoGoaliePosition) // control conditions, too restrictive
    {
        transition
        {
#ifdef SPQR_DEBUG_GOALIE			
			std::cerr << "gotogoalieposition" << std::endl;
#endif			

            if( libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.x, libCodeRelease.getGoalieCoverPosition().translation.x() +libCodeRelease.goalie_displacement,
                                               SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                    libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.y, libCodeRelease.getGoalieCoverPosition().translation.y(),
                                                   SPQR::GOALIE_POSE_Y_TOLLERANCE) )
                goto turnToOpponentGoal;

            if( libCodeRelease.isGoalieInAngle() )
                goto backTrackInPose;

        }
        action
        {
            if(std::abs(libCodeRelease.angleToTarget(libCodeRelease.getGoalieCoverPosition().translation.x() +libCodeRelease.goalie_displacement,
                                                     libCodeRelease.getGoalieCoverPosition().translation.y())) > Angle::fromDegrees(10.f))
            {
				
#ifdef SPQR_GOALIE_LOOK_AROUND
				theHeadControlMode = HeadControl::lookLeftAndRight;
#else
                theHeadControlMode = HeadControl::lookForwardGoalie;
#endif
                WalkToTarget(Pose2f(50.f, 0.f, 0.f),
                             Pose2f(libCodeRelease.angleToTarget(libCodeRelease.getGoalieCoverPosition().translation.x() +libCodeRelease.goalie_displacement,
                                                                 libCodeRelease.getGoalieCoverPosition().translation.y()), 0.f, 0.f));
            }
            else
            {
#ifdef SPQR_GOALIE_LOOK_AROUND
				theHeadControlMode = HeadControl::lookLeftAndRight;
#else
                theHeadControlMode = HeadControl::lookForwardGoalie;
#endif                
                WalkToTarget(Pose2f(.2f, SPQR::WALKING_VELOCITY_X, 20.f),
                             libCodeRelease.glob2Rel(libCodeRelease.getGoalieCoverPosition().translation.x() +libCodeRelease.goalie_displacement,
                              libCodeRelease.getGoalieCoverPosition().translation.y()));
            }
        }

    }

    state(backTrackInPose)
    {
        transition
        {
#ifdef SPQR_DEBUG_GOALIE
			std::cerr << "backtrackinpose" << std::endl;
#endif
			// Tuffati anche mentre torni indietro
            if(std::abs(theDiveHandle.ballProjectionEstimate) < SPQR::GOALIE_FAR_LIMIT_Y)
            {
                if( theDiveHandle.diveTime != -1 && theDiveHandle.diveTime < SPQR::GOALIE_DIVE_TIME_TOLERANCE )
                {
					
                    SPQR_INFO("[Goalie.h] dive time: " << theDiveHandle.diveTime);
                    if(theDiveHandle.ballProjectionEstimate > SPQR::GOALIE_CLOSE_LIMIT_Y)
                        goto goalieDiveLeft;
                    else if(theDiveHandle.ballProjectionEstimate < -SPQR::GOALIE_CLOSE_LIMIT_Y)
                        goto goalieDiveRight;
                    else
                        goto stopBall;
                }
            }
#ifdef SPQR_DEBUG_GOALIE
			std::cerr << "backtrack balanced" <<  (libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.x, libCodeRelease.getGoalieCoverPosition().translation.x(), SPQR::GOALIE_POSE_X_TOLLERANCE) &&
						libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.y, libCodeRelease.getGoalieCoverPosition().translation.y(), SPQR::GOALIE_POSE_Y_TOLLERANCE)) << std::endl;
#endif                    
            if( libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.x, libCodeRelease.getGoalieCoverPosition().translation.x(), SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                    libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.y, libCodeRelease.getGoalieCoverPosition().translation.y(), SPQR::GOALIE_POSE_Y_TOLLERANCE) )
                //~ if( libCodeRelease.isGoalieInAngle() )
                    goto mainLoop;
#ifdef SPQR_DEBUG_GOALIE
			std::cerr << "backtrack " << libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getGoalieCoverPosition().translation.x(),
						theRobotPoseSpqrFiltered.y - libCodeRelease.getGoalieCoverPosition().translation.y()) << std::endl;
#endif                                    
#ifndef SPQR_GOALIE_NOT_BACKTRACK
            if( libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getGoalieCoverPosition().translation.x(),
                                    theRobotPoseSpqrFiltered.y - libCodeRelease.getGoalieCoverPosition().translation.y()) > 800
                                     )
                goto gotoGoaliePosition;
#endif
            if (!libCodeRelease.isGoalieInAngle() ) {
				goto turnToOpponentGoal;
			}

        }
        action
        {
#ifdef SPQR_GOALIE_LOOK_AROUND
			theHeadControlMode = HeadControl::lookLeftAndRight;
#else			
            theHeadControlMode = HeadControl::lookForwardGoalie;
#endif            
            if( !libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.x, libCodeRelease.getGoalieCoverPosition().translation.x(), SPQR::GOALIE_POSE_X_TOLLERANCE) 
					|| !libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.y, libCodeRelease.getGoalieCoverPosition().translation.y(), SPQR::GOALIE_POSE_Y_TOLLERANCE) )
                WalkToTarget(Pose2f(.0f, 50.f, 50.f), libCodeRelease.glob2Rel(libCodeRelease.getGoalieCoverPosition().translation.x(), libCodeRelease.getGoalieCoverPosition().translation.y()));
        }
    }

    state(turnToOpponentGoal) // Actually turn to the "right" position according to the ball position.
    {
        transition
        {
#ifdef SPQR_DEBUG_GOALIE			
			std::cerr << "turntoopponentgoal" << std::endl;
#endif			
            if( libCodeRelease.isGoalieInAngle() )
                goto mainLoop;
        }
        action
        {
#ifdef SPQR_GOALIE_LOOK_AROUND
			theHeadControlMode = HeadControl::lookLeftAndRight;
#else			
            theHeadControlMode = HeadControl::lookForwardGoalie;
#endif 
            OUTPUT(idText, text, libCodeRelease.getGoalieCoverAngleDisplacement());
            if( libCodeRelease.getGoalieCoverAngleDisplacement() > 0.17f)
                WalkAtSpeedPercentage(Pose2f(-.6f, 0.f, 0.f));
            else if ( libCodeRelease.getGoalieCoverAngleDisplacement() < -0.17f)
                WalkAtSpeedPercentage(Pose2f(.6f, 0.f, 0.f));
        }
    }

    state(mainLoop)
    {
        transition
        {
            if( theBallModel.estimate.velocity.norm() < SPQR::MOVING_BALL_MIN_VELOCITY // norm < 1000
                            && libCodeRelease.isBallInKickAwayRange()
                            && theGameInfo.state == STATE_PLAYING
                            && libCodeRelease.isBallInArea()
                            && theBallModel.estimate.position.x() != 0.0
                            && theBallModel.estimate.position.y() != 0.0
                            && libCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() < -3900 )
            {
                    goto goalieKickAway;
            }
#ifdef SPQR_DEBUG_GOALIE			
			std::cerr << "mainloop" << std::endl;
#endif			
#ifndef SPQR_GOALIE_STAND				
 #ifdef SPQR_DEBUG_GOALIE
 			std::cerr << "back : " << libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getGoalieCoverPosition().translation.x(),
 					theRobotPoseSpqrFiltered.y - libCodeRelease.getGoalieCoverPosition().translation.y()) << std::endl;
 #endif                                    
             if( libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getGoalieCoverPosition().translation.x(),
                                     theRobotPoseSpqrFiltered.y - libCodeRelease.getGoalieCoverPosition().translation.y()) > 50 )
                                     {
                 goto backTrackInPose;
                 }
 #ifdef SPQR_DEBUG_GOALIE
 			std::cerr << "goto : " << libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getGoalieCoverPosition().translation.x(),
 					theRobotPoseSpqrFiltered.y - libCodeRelease.getGoalieCoverPosition().translation.y()) << std::endl;
 #endif
 #ifndef SPQR_GOALIE_NOT_BACKTRACK                              
             if( libCodeRelease.norm(theRobotPoseSpqrFiltered.x - libCodeRelease.getGoalieCoverPosition().translation.x(),
                                     theRobotPoseSpqrFiltered.y - libCodeRelease.getGoalieCoverPosition().translation.y()) > 500 )
                                     {
                 goto gotoGoaliePosition;
 			}
 #endif
			if (!libCodeRelease.isGoalieInAngle()) {
				goto turnToOpponentGoal;
			}
#endif
			
			// Test can be harmful
			//~ if (libCodeRelease.norm(thePenaltyMarkPercept.positionOnField.x(), thePenaltyMarkPercept.positionOnField.y()) > 1300
					//~ && thePenaltyMarkPercept.timeLastSeen < 1000) {
						//~ std::cerr << "pen dist" << libCodeRelease.norm(thePenaltyMarkPercept.positionOnField.x(), thePenaltyMarkPercept.positionOnField.y())<< std::endl;
				//~ goto walkAhead;
			//~ }


            if(std::abs(theDiveHandle.ballProjectionEstimate) < SPQR::GOALIE_FAR_LIMIT_Y)
            {
                if( theDiveHandle.diveTime != -1 && theDiveHandle.diveTime < SPQR::GOALIE_DIVE_TIME_TOLERANCE )
                {
                    SPQR_INFO("[Goalie.h] dive time: " << theDiveHandle.diveTime);
                    if(theDiveHandle.ballProjectionEstimate > SPQR::GOALIE_CLOSE_LIMIT_Y)
                        goto goalieDiveLeft;
                    else if(theDiveHandle.ballProjectionEstimate < -SPQR::GOALIE_CLOSE_LIMIT_Y)
                        goto goalieDiveRight;
                    else
                        goto stopBall;
                }
            }
        }
        action
        {
            if(libCodeRelease.timeSinceBallWasSeen() < 2000)
                theHeadControlMode = HeadControl::lookAtBall;
            //~ else if(theSpqrDWKcombiner.current_context == SpqrDWKcombiner::playing)
                //~ theHeadControlMode = HeadControl::lookAtGlobalBall;
            else
                theHeadControlMode = HeadControl::lookLeftAndRight;
            Stand();
        }
    }
    
    state(walkAhead) 
    {
		transition
		{
			if (state_time > 3000)
				goto mainLoop;
		}
		action
		{
                WalkAtSpeedPercentage(Pose2f(.0f, 0.5f, 0.f));
		}
	}
    
    state(goalieKickAway)
    {
        transition
        {
						theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_KEEPER;

#ifdef SPQR_DEBUG_GOALIE			
			std::cerr << "goaliekickaway" << std::endl;
			std::cerr << "ballsinceseen : " << libCodeRelease.timeSinceBallWasSeen() << std::endl;
			std::cerr << "state time : " << state_time << std::endl;
#endif			
            if(!libCodeRelease.isGoalieInArea() || action_done || !libCodeRelease.isBallInKickAwayRange() || state_time > 10000 || libCodeRelease.timeSinceBallWasSeen() > 500 ) // senza state_time
                goto gotoGoaliePosition;

            if( libCodeRelease.timeSinceBallWasSeen() > 3000 && !libCodeRelease.isBallInKickAwayRange() )
                goto mainLoop;
			

        }
        action
        {

            if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                theHeadControlMode = HeadControl::lookLeftAndRight;
            else
                //~ theHeadControlMode = HeadControl::lookAtGlobalBall;
                theHeadControlMode = HeadControl::lookAtBall;
                std::cerr << "seen ball at : " << theBallModel.estimate.position.x() << ", " << theBallModel.estimate.position.y() << std::endl;
				GoalieKickAway();
        }
    }

    state(goalieDiveLeft)
    {
        transition
        {
#ifdef SPQR_GOALIE_STAND
            if (state_time > SPQR::GOALIE_DIVE_TIME)
				goto mainLoop;
#endif
#ifndef SPQR_GOALIE_STAND
            if (state_time > SPQR::GOALIE_DIVE_TIME)
                goto gotoGoaliePosition;
#endif
        }
        action
        {
            theHeadControlMode = HeadControl::lookForwardGoalie;
            GoalieSaveLeft();
        }
    }

    state(goalieDiveRight)
    {
        transition
        {
#ifdef SPQR_GOALIE_STAND
            if (state_time > SPQR::GOALIE_DIVE_TIME)
				goto mainLoop;
#endif
#ifndef SPQR_GOALIE_STAND
            if (state_time > SPQR::GOALIE_DIVE_TIME)
                goto gotoGoaliePosition;
#endif
        }
        action
        {
            theHeadControlMode = HeadControl::lookForwardGoalie;
            GoalieSaveRight();
        }
    }

    state(stopBall)
    {
        transition
        {
#ifdef SPQR_GOALIE_STAND
            if (state_time > SPQR::GOALIE_DIVE_TIME)
				goto mainLoop;
#endif            
#ifndef SPQR_GOALIE_STAND
            if (state_time > SPQR::GOALIE_DIVE_TIME)
                goto gotoGoaliePosition;
#endif
        }
        action
        {
            theHeadControlMode = HeadControl::lookForwardGoalie;
            StopBall();
        }
    }

}
