
//#define SPQR_DEBUG_STRIKER
//#define NOT_KICKING_STRIKER

#define DRIBBLE_BALL_UPPER_THRESHOLD 0.15f
#define DRIBBLE_BALL_LOWER_THRESHOLD -0.50f

option(Striker)
{

    common_transition
    {
        //        if(theBallModel.estimate.position.x() < 170.f && std::fabs(theBallModel.estimate.position.y()) < 150.f &&
        //                std::abs(libCodeRelease.angleToGoal) < libCodeRelease.kickAngle && theRobotPoseSpqrFiltered.theta < Angle::fromDegrees(45.f)
        //                && state_time > 7000 )
        //            goto kick;

#ifdef AUDIO_IN_BEHAVIORS
        if (option_time < 10)
        {
            std::string wavName = Global::getSettings().robotName.c_str();
            wavName.append(".wav");
            SystemCall::playSound(wavName.c_str());
            SystemCall::playSound("Striker.wav");
        }
#endif
    }

    initial_state(start)
    {
        transition
        {
            if( libCodeRelease.timeSinceBallWasSeen() > 3000 )
                goto lookAround;
            else goto walkToBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            Stand();
        }
    }

    state(kick_off)
    {
        transition
        {
            if( libCodeRelease.timeSinceBallWasSeen() > 3000 )
                goto lookAround;

            if( theSpqrDWKcombiner.estimated_ball_global.norm() > 800.f ) goto walkToBall;

        }
        action
        {
            //            if( theSpqrDWKcombiner.estimated_ball_global.norm() > 800.f )
            //                    theHeadControlMode = HeadControl::lookLeftAndRightSmallAngles;
            //            else
            theHeadControlMode = HeadControl::lookAtBall;
            KickOffPushBallOverCenter();
        }
    }

    state(lookAround)
    {
        transition
        {
            if(theSpqrDWKcombiner.timeSinceWasSeen < 5000)
                goto walkToGlobalBall;

            if(libCodeRelease.timeSinceBallWasSeen() < 300){
                goto walkToBall;
            }

        }

        action
        {
            theHeadControlMode = HeadControl::lookForward;
            if( libCodeRelease.ballOutOnLeft )
                WalkAtSpeedPercentage(Pose2f(-.8f, 0.f, 0.f));
            else
                WalkAtSpeedPercentage(Pose2f(.8f, 0.f, 0.f));

            //            if(libCodeRelease.timeSinceBallWasSeen() > 300){
            //                SystemCall::playSound("iDontSeeTheBall.wav");
            //            }
        }
    }

    state(walkToGlobalBall)
    {
        transition
        {

            //if(libCodeRelease.timeSinceBallWasSeen() < 500)
                goto walkToBall;
        }
        action
        {
            Vector2f glob_ball_rel = libCodeRelease.glob2Rel(theSpqrDWKcombiner.estimated_ball_global.x(), theSpqrDWKcombiner.estimated_ball_global.y()).translation;

            if( theSpqrDWKcombiner.estimated_ball_global.norm() > 1000.f || theBallModel.estimate.position.norm() > 1000.f)
            {
                //cout << state_time%2000 << "  " << ((state_time%4000 < 2000 ) ? "RightLeft": "landmarks") << endl;
                if(theSpqrDWKcombiner.estimated_ball_global.norm() < 2500.f || theBallModel.estimate.position.norm() < 2500.f)
                    theHeadControlMode = HeadControl::lookLeftAndRight/*ForStriker*/;
                else
                    theHeadControlMode = HeadControl::lookAtLandmark;
            }
            else
            {
                theHeadControlMode = HeadControl::lookAtGlobalBall;
            }

            if(std::norm(glob_ball_rel.angle()) > Angle::fromDegrees(10.f))
                WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(glob_ball_rel.angle(), 0.f, 0.f));
            else
                WalkToTarget( Pose2f(.2f, SPQR::WALKING_VELOCITY_X, 10.f), Pose2f(glob_ball_rel - Vector2f(200.0f, .0f)) );
        }
    }

    state(turnLeft)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() < 300)
            {
                //   SystemCall::playSound("iSeeTheBall.wav");
                goto walkToBall;
            }
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            WalkAtSpeedPercentage(Pose2f(1.f, 0.f, 0.f));
            //SystemCall::playSound("iDontSeeTheBall.wav");

        }
    }

    state(turnRight)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() < 300)
            {
                //    SystemCall::playSound("iSeeTheBall.wav");
                goto walkToBall;
            }
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            WalkAtSpeedPercentage(Pose2f(-1.f, 0.f, 0.f));
            //    SystemCall::playSound("iDontSeeTheBall.wav");

        }
    }

    state(walkToBall)
    {
        transition
        {
            if( theBallModel.estimate.position.norm() < 400.f )
                goto alignToGoal;

            if( libCodeRelease.timeSinceBallWasSeen() > 3000 )
                goto lookAround;
        }
        action
        {
//            if( theSpqrDWKcombiner.estimated_ball_global.norm() > 1000.f || theBallModel.estimate.position.norm() > 1000.f)
//            {
//                //                cout << state_time%2000 << "  " << ((state_time%4000 < 2000 ) ? "RightLeft": "landmarks") << endl;
//                if( theSpqrDWKcombiner.estimated_ball_global.norm() < 2500.f || theBallModel.estimate.position.norm() < 2500.f)
//                    theHeadControlMode = HeadControl::lookLeftAndRightForStriker;
//                else
//                    theHeadControlMode = HeadControl::lookAtLandmark;
//            }
//            else
            {
                theHeadControlMode = HeadControl::lookAtBall;
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
//                    theBallModel.estimate.position.x() < 100.0f ||
                    std::fabs(theBallModel.estimate.position.y()) > 250.0f )
                goto walkToBall;

#ifndef NOT_KICKING_STRIKER
            if( libCodeRelease.kickToPerform() == "sideKick") goto prepareSideKick;

//            if( libCodeRelease.kickToPerform() == "backKick" &&
//                    theSpqrDWKcombiner.estimated_ball_global.x() > 0.2*SPQR::FIELD_DIMENSION_X ) goto backKick;

//            else if( libCodeRelease.kickToPerform() == "extSideKick") goto extSideKick;
#endif

            // dribble ball condition
            if( theRobotPoseSpqrFiltered.x > DRIBBLE_BALL_LOWER_THRESHOLD*SPQR::FIELD_DIMENSION_X &&
                    theRobotPoseSpqrFiltered.x <= DRIBBLE_BALL_UPPER_THRESHOLD*SPQR::FIELD_DIMENSION_X)
                goto dribbleBallAhead;

            if(std::fabs(libCodeRelease.angleToGoal) < libCodeRelease.kickAngle &&
                    //std::fabs(theRobotPoseSpqrFiltered.theta)<Angle::fromDegrees(45) &&       //CHECK Vincenzo
                    std::fabs(theBallModel.estimate.position.y()) < 180.f)
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

    state(prepareSideKick)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto lookAround;

            if(theBallModel.estimate.position.norm() > 300.f || theBallModel.estimate.position.y() > 200)
                goto walkToBall;

            if(((/*libCodeRelease.between(std::abs(libCodeRelease.angleToGoal), Angle::fromDegrees(80.f), Angle::fromDegrees(100.f)) &&*/
                 theBallModel.estimate.position.x() < 180.f ) /*|| state_time > 6000*/)&&
                    libCodeRelease.between(std::abs(libCodeRelease.angleToGoal), Angle::fromDegrees(65.f)-libCodeRelease.correctionKickAngle, Angle::fromDegrees(75.f)+libCodeRelease.correctionKickAngle))
                goto sideKick;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            /* if(theRobotPoseSpqrFiltered.theta > .0f)
                WalkToTarget( Pose2f(.2f, 20.f, 20.f), theBallModel.estimate.position - Vector2f(150.0f, -10.f) );
            else
                WalkToTarget( Pose2f(.2f, 20.f, 20.f), theBallModel.estimate.position - Vector2f(150.0f, 10.f) );
            */
            if(theRobotPoseSpqrFiltered.theta > .0f)
                WalkToTarget( Pose2f(.2f, 20.f, 20.f), Pose2f(libCodeRelease.angleToGoal+Angle::fromDegrees(70.f)+0.6f*libCodeRelease.correctionKickAngle,
                                                              theBallModel.estimate.position.x() - 170.f, theBallModel.estimate.position.y()+10.f));
            else
                WalkToTarget( Pose2f(.2f, 20.f, 20.f), Pose2f(libCodeRelease.angleToGoal-Angle::fromDegrees(70.f)-0.6*libCodeRelease.correctionKickAngle,
                                                              theBallModel.estimate.position.x() - 170.f, theBallModel.estimate.position.y()-10.f));

        }
    }

    state(sideKick)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() > 300.f || theBallModel.estimate.position.y() > 200)
                goto walkToBall;

            if( (state_time > 2000 || (state_time > 10 && action_done)) &&
                    libCodeRelease.glob2Rel(SPQR::FIELD_DIMENSION_X, .0f).translation.y() > .0f)
                goto turnLeft;
            else if( (state_time > 2000 || (state_time > 10 && action_done)) &&
                     libCodeRelease.glob2Rel(SPQR::FIELD_DIMENSION_X, .0f).translation.y() <= .0f)
                goto turnRight;
            else if( state_time > 2000 || (state_time > 100 && action_done) )
                goto lookAround;
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            Kicks("sideKick");
        }
    }

    state(alignBehindBall)
    {
        transition
        {
            if(!libCodeRelease.isTheAreaCleanFromOpponents(1200))
                goto alignBehindBallFast;

            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto lookAround;

            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(50.f) /*&& theBallModel.estimate.position.x() < 50.0f*/
                    || std::abs(theBallModel.estimate.position.y())> 250.f)
                goto walkToBall;


            // y condition
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 0.f, 110.f) ||  //30,50
                    libCodeRelease.between(theBallModel.estimate.position.y(), -110.f, -0.f) ) //-50, -30
            {
                // x condition
                if( libCodeRelease.between(theBallModel.estimate.position.x(), 30.f, 180.f) &&
                        std::abs(libCodeRelease.angleToGoal) < libCodeRelease.kickAngle )
                {

                    if( libCodeRelease.dribbleAnOpponent() && 0 ) goto alignDribble;
                    //
                    // dribble ball condition
                    if( theRobotPoseSpqrFiltered.x > DRIBBLE_BALL_LOWER_THRESHOLD*SPQR::FIELD_DIMENSION_X &&
                            theRobotPoseSpqrFiltered.x <= DRIBBLE_BALL_UPPER_THRESHOLD*SPQR::FIELD_DIMENSION_X)
                        goto dribbleBallAhead;

#ifndef NOT_KICKING_STRIKER
                    //if(!isTheAreaCleanFromOpponents(500))
                    //    gotofastAndVeryFastKicks
                    goto kick;
#endif
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

    state(alignBehindBallFast)
    {
        transition
        {
           if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto lookAround;

            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(50.f) /*&& theBallModel.estimate.position.x() < 50.0f*/
                    || std::abs(theBallModel.estimate.position.y())> 250.f)
                goto walkToBall;

            // y condition
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 0.f, 50.f) ||  //30,50
                    libCodeRelease.between(theBallModel.estimate.position.y(), -50.f, -0.f) ) //-50, -30
            {
                // x condition
                if( libCodeRelease.between(theBallModel.estimate.position.x(), 30.f, 200.f) &&
                        std::abs(libCodeRelease.angleToGoal) < libCodeRelease.kickAngle )
                {

                    if( libCodeRelease.dribbleAnOpponent() && 0 ) goto alignDribble;
                    // dribble ball condition
                    if( theRobotPoseSpqrFiltered.x > DRIBBLE_BALL_LOWER_THRESHOLD*SPQR::FIELD_DIMENSION_X &&
                            theRobotPoseSpqrFiltered.x <= DRIBBLE_BALL_UPPER_THRESHOLD*SPQR::FIELD_DIMENSION_X)
                        goto dribbleBallAhead;

#ifndef NOT_KICKING_STRIKER
                    goto fastAndVeryFastKicks;
#endif
                }
            }
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            if( theBallModel.estimate.position.y() >= .0f )
                WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                             Pose2f(libCodeRelease.angleToGoal+libCodeRelease.correctionKickAngle,
                                    theBallModel.estimate.position.x() - 180.f,
                                    theBallModel.estimate.position.y() - 50.f));
            else
                WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                             Pose2f(libCodeRelease.angleToGoal-libCodeRelease.correctionKickAngle,
                                    theBallModel.estimate.position.x() - 180.f,
                                    theBallModel.estimate.position.y() + 50.f));
        }
    }

    state(dribbleBallAhead)
    {
        transition
        {
            //             if( libCodeRelease.dribbleAnOpponent() ) goto alignDribble;

            if( libCodeRelease.timeSinceBallWasSeen() > 3000 )
                goto lookAround;

            if( theBallModel.estimate.position.norm() > 300.f )
                goto walkToBall;

            // kick condition
            if( !(theRobotPoseSpqrFiltered.x > DRIBBLE_BALL_LOWER_THRESHOLD*SPQR::FIELD_DIMENSION_X &&
                  theRobotPoseSpqrFiltered.x <= DRIBBLE_BALL_UPPER_THRESHOLD*SPQR::FIELD_DIMENSION_X) )
                goto alignBehindBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTarget( Pose2f(.2f, SPQR::WALKING_VELOCITY_X, 30.f), Pose2f(theBallModel.estimate.position + Vector2f(150.0f, .0f)) );
        }
    }

    state(alignDribble)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto lookAround;

            if(theBallModel.estimate.position.norm() > 400.f)
                goto walkToBall;

//            if( libCodeRelease.isDribbleBallPosition() &&
//                    libCodeRelease.between(std::fabs(libCodeRelease.angleToGoal),
//                                           std::fabs(Angle::fromDegrees(65.f)-libCodeRelease.correctionKickAngle),
//                                           std::fabs(Angle::fromDegrees(75.f)+libCodeRelease.correctionKickAngle)) ) goto dribbleKick;

            if(std::fabs(libCodeRelease.angleToGoal) < libCodeRelease.kickAngle &&
                    std::fabs(theRobotPoseSpqrFiltered.theta)<Angle::fromDegrees(45) &&
                    std::fabs(theBallModel.estimate.position.y()) < 150.f)
                goto alignBehindBall;

            if(theRobotPoseSpqrFiltered.theta < Angle::fromDegrees(15.f) || theRobotPoseSpqrFiltered.theta > Angle::fromDegrees(160.f))
                goto alignToGoal;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            if(theRobotPoseSpqrFiltered.theta < .0f)
                WalkToTarget( Pose2f(.2f, SPQR::WALKING_VELOCITY_X, 30.f),
                              Pose2f(libCodeRelease.angleToGoal+Angle::fromDegrees(70.f)+0.6f*libCodeRelease.correctionKickAngle,
                                     theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y()+7.f));
            else
                WalkToTarget( Pose2f(.2f, SPQR::WALKING_VELOCITY_X, 30.f),
                              Pose2f(libCodeRelease.angleToGoal-Angle::fromDegrees(70.f)-0.6*libCodeRelease.correctionKickAngle,
                                     theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y()-7.f));
        }
    }

//    state(dribbleKick)
//    {
//        transition
//        {
//            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
//                goto lookAround;

//            if( (state_time > 10000 || (state_time > 10 && action_done)) &&
//                    libCodeRelease.glob2Rel(SPQR::FIELD_DIMENSION_X, .0f).translation.y() > .0f)
//                goto turnLeft;
//            else if( (state_time > 10000 || (state_time > 10 && action_done)) &&
//                     libCodeRelease.glob2Rel(SPQR::FIELD_DIMENSION_X, .0f).translation.y() <= .0f)
//                goto turnRight;

//            if(theBallModel.estimate.position.norm() > 250.f && action_done)
//                goto walkToBall;
//        }
//        action
//        {
//            theHeadControlMode = HeadControl::lookAtBall;

//            if( libCodeRelease.dribbleAnOpponent() && 0)
//            {
//                if( theRobotPoseSpqrFiltered.y > 0.f )
//                    InDribbleKick(WalkRequest::sidewardsLeft,
//                                  Pose2f(theRobotPose.rotation, theBallModel.estimate.position.x() , theBallModel.estimate.position.y()));
//                else
//                    InDribbleKick(WalkRequest::sidewardsRight,
//                                  Pose2f(theRobotPose.rotation, theBallModel.estimate.position.x() , theBallModel.estimate.position.y()));
//            }
//            else
//            {
//                if( theRobotPoseSpqrFiltered.y <= theSpqrDWKcombiner.estimated_ball_global.y() )
//                    InDribbleKick(WalkRequest::sidewardsLeft,
//                                  Pose2f(theRobotPose.rotation, theBallModel.estimate.position.x() , theBallModel.estimate.position.y()));
//                else
//                    InDribbleKick(WalkRequest::sidewardsRight,
//                                  Pose2f(theRobotPose.rotation, theBallModel.estimate.position.x() , theBallModel.estimate.position.y()));
//            }
//        }
//    }

    state(backKick)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto lookAround;

            if(theBallModel.estimate.position.norm() > 500.f)
                goto walkToBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            if( theBallModel.estimate.position.x() <= SPQR::BACKKICK_APPROACH_X &&
                    std::fabs(theBallModel.estimate.position.y()) <= SPQR::BACKKICK_APPROACH_Y &&
                    (std::abs(libCodeRelease.angleToGoal - Angle::fromDegrees(SPQR::BACKKICK_ANGLE) + libCodeRelease.correctionKickAngle)
                     <Angle::fromDegrees(5.f) ||
                     std::abs(libCodeRelease.angleToGoal + Angle::fromDegrees(SPQR::BACKKICK_ANGLE) - libCodeRelease.correctionKickAngle)
                     <Angle::fromDegrees(5.f)) )
                Kicks("backKick");
            else
            {
                if(theRobotPoseSpqrFiltered.theta < .0f)
                    WalkToTarget( Pose2f(.2f, 10.f, 10.f),
                                  Pose2f(libCodeRelease.angleToGoal-Angle::fromDegrees(SPQR::BACKKICK_ANGLE)+
                                         0.6f*libCodeRelease.correctionKickAngle,
                                         theBallModel.estimate.position.x() - SPQR::BACKKICK_APPROACH_X*0.95,
                                         theBallModel.estimate.position.y() - SPQR::BACKKICK_APPROACH_Y));
                else
                    WalkToTarget( Pose2f(.2f, 10.f, 10.f),
                                  Pose2f(libCodeRelease.angleToGoal+Angle::fromDegrees(SPQR::BACKKICK_ANGLE)-
                                         0.6*libCodeRelease.correctionKickAngle,
                                         theBallModel.estimate.position.x() - SPQR::BACKKICK_APPROACH_X*0.95,
                                         theBallModel.estimate.position.y() + SPQR::BACKKICK_APPROACH_Y));
            }
        }
    }

    state(extSideKick)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto lookAround;

            if(theBallModel.estimate.position.norm() > 500.f)
                goto walkToBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;

            if(((libCodeRelease.between(std::abs(libCodeRelease.angleToGoal), Angle::fromDegrees(80.f), Angle::fromDegrees(100.f)) &&
                 theBallModel.estimate.position.x() < 2.0f ) && (theBallModel.estimate.position.y() < 150.f) )|| state_time > 6000)
                Kicks("extSideKick");
            else
            {
                if(theRobotPoseSpqrFiltered.theta > .0f)
                    WalkToTarget( Pose2f(.0f, 20.f, 20.f), Pose2f(theBallModel.estimate.position - Vector2f(2.0f, -150.f)) );
                else
                    WalkToTarget( Pose2f(.0f, 20.f, 20.f), Pose2f(theBallModel.estimate.position - Vector2f(2.0f, 150.f)) );
            }
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

#ifndef NOT_KICKING_STRIKER
            //Kicks("forwardKick");
            Kicks("fastForwardKick");
#else
            Stand();
#endif

        }
    }

    state(fastAndVeryFastKicks) //if (there are opposite robots around the striker) fastforwardkick or veryfastforwardKick
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
#ifndef NOT_KICKING_STRIKER
            if(!libCodeRelease.isTheAreaCleanFromOpponents(300))
                Kicks("VeryFastForwardKick");
            else
                Kicks("fastForwardKick");
#else
            Stand();
#endif
        }
    }


    state(fastKick)
    {
        transition
        {
#ifdef SPQR_DEBUG_STRIKER
            STATE("fastKick");
#endif
            if(state_time > 3000 || (state_time > 10 && action_done))
                goto lookAround;

            if(theBallModel.estimate.position.norm() > 500.f)
                goto walkToBall;
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            if( theBallModel.estimate.position.y() >= .0f )
                InWalkKick(WalkRequest::left,
                           Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 160.f,
                                  theBallModel.estimate.position.y() - 40.f));
            else
                InWalkKick(WalkRequest::right,
                           Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 160.f,
                                  theBallModel.estimate.position.y() + 40.f));

        }
    }

    //    state(waitForGoaliekickAway)
    //    {
    //        transition
    //        {
    //#ifdef SPQR_DEBUG_STRIKER
    //            STATE("waitForGoaliekickAway");
    //#endif
    //            if( libCodeRelease.timeSinceBallWasSeen() > 2000 )
    //                goto lookAround;

    //            //penalty area bounds
    //            if(theGlobalBallEstimation.singleRobotX > -SPQR::FIELD_DIMENSION_X + SPQR::PENALTY_AREA_X ||
    //                    std::abs(theGlobalBallEstimation.singleRobotY)> SPQR::PENALTY_AREA_Y )
    //                goto walkToBall;

    //        }
    //        action

    //        {
    //            theHeadControlMode = HeadControl::lookAtBall;

    //            if( std::abs(theRobotPoseSpqrFiltered.y - theGlobalBallEstimation.singleRobotY) < 300.f  )
    //            {
    //                if(theBallModel.estimate.position.y() > .0f)
    //                    WalkToTarget( Pose2f(.0f, 10.f, 10.f),
    //                                  libCodeRelease.glob2Rel(-SPQR::FIELD_DIMENSION_X + SPQR::PENALTY_AREA_X + 150,
    //                                                          theGlobalBallEstimation.singleRobotY +350).translation );
    //                else
    //                    WalkToTarget( Pose2f(.0f, 10.f, 10.f),
    //                                  libCodeRelease.glob2Rel(-SPQR::FIELD_DIMENSION_X + SPQR::PENALTY_AREA_X + 150,
    //                                                          theGlobalBallEstimation.singleRobotY -350).translation );
    //            }
    //            else if( std::abs(theRobotPoseSpqrFiltered.y - theGlobalBallEstimation.singleRobotY) > 290.f &&
    //                     theRobotPoseSpqrFiltered.y >= theGlobalBallEstimation.singleRobotY &&
    //                     !libCodeRelease.between(theRobotPoseSpqrFiltered.theta, Angle::fromDegrees(-120.f), Angle::fromDegrees(-100.f)) )
    //                WalkAtSpeedPercentage(Pose2f(.6f, 0.f, 0.f));

    //            else if( std::abs(theRobotPoseSpqrFiltered.y - theGlobalBallEstimation.singleRobotY) > 290.f &&
    //                     theRobotPoseSpqrFiltered.y < theGlobalBallEstimation.singleRobotY &&
    //                     !libCodeRelease.between(theRobotPoseSpqrFiltered.theta, Angle::fromDegrees(100.f), Angle::fromDegrees(120.f)) )
    //                WalkAtSpeedPercentage(Pose2f(-.6f, 0.f, 0.f));

    //            else
    //                Stand();
    //        }
    //    }

}
