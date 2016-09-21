////option(PassBall)
////{
////    initial_state(start)
////    {
////        transition
////        {
////            goto beStationary;
////        }
////        action
////        {
////            theHeadControlMode = HeadControl::lookLeftAndRight;
////            Stand();
////        }
////    }

////    state(beStationary)
////    {
////        transition
////        {
////            if (((Vector2f)(libCodeRelease.getTeammatePosition() - theBallModel.estimate.position)).norm()
////                    > theBallModel.estimate.position.norm())
////                goto kickAway;
////            if (std::abs(libCodeRelease.getAngleToTeammate()) > Angle::fromDegrees(4.f))
////                goto alignToOtherRobot;
////            //            if (!libCodeRelease.between(libCodeRelease.getTeammatePosition().norm(),1500, 1800))
////            //                goto keepDistance;
////        }
////        action
////        {
////            if(theFrameInfo.time - theBallModel.timeWhenLastSeen < 1000)
////                theHeadControlMode = HeadControl::lookAtBall;
////            else
////                theHeadControlMode = HeadControl::lookLeftAndRight/*UpAndDown*/;
////            //~
////            Stand();
////        }
////    }

////    //    state(keepDistance)
////    //    {
////    //        transition
////    //        {
////    //            if (std::abs(libCodeRelease.getAngleToTeammate()) > Angle::fromDegrees(10.f))
////    //                goto alignToOtherRobot;
////    //            if (libCodeRelease.between(libCodeRelease.getTeammatePosition().norm(),1500, 1800))
////    //                goto beStationary;
////    //        }

////    //        action
////    //        {
////    //            if (libCodeRelease.getTeammatePosition().norm() > 2000)
////    //                WalkAtSpeedPercentage(Pose2f(.0f, .5f, .0f));
////    //            else
////    //                WalkAtSpeedPercentage(Pose2f(.0f, -.5f, .0f));
////    //        }
////    //    }

////    state(kickAway)
////    {
////        transition
////        {
////            if (theBallModel.estimate.position.norm() > 300.f)
////                goto turnToBall;
////            if (theBallModel.estimate.position.norm() < 300.f)
////                goto walkToBall;
////            if (libCodeRelease.timeSinceBallWasSeen() > 1000)
////                goto beStationary;

////        }
////        action
////        {
////            if (libCodeRelease.timeSinceBallWasSeen() > 1000)
////                theHeadControlMode = HeadControl::lookLeftAndRight;
////            else
////                theHeadControlMode = HeadControl::lookAtBall;
////            Stand();
////        }
////    }

////    state(alignToOtherRobot)
////    {
////        transition
////        {

////            if(std::abs(libCodeRelease.getAngleToTeammate()) < Angle::fromDegrees(4.f))
////            {
////                if (std::abs(theBallModel.estimate.position.y()) < 150.f && std::abs(theBallModel.estimate.position.x()) < 250.f)
////                    goto alignBehindBall;
////                else
////                    goto beStationary;
////            }

////            if (!libCodeRelease.between(fabs(theBallModel.estimate.position.angle()), Angle::fromDegrees(0), Angle::fromDegrees(90) ))
////                goto turnToBall;
////        }
////        action
////        {
////            theHeadControlMode = HeadControl::lookLeftAndRight;
////            WalkToTarget(Pose2f(0.4f, 10.f, 30.f), Pose2f(libCodeRelease.getAngleToTeammate(), theBallModel.estimate.position.x() - 300.f, theBallModel.estimate.position.y()));
////        }
////    }

////    state(alignBehindBall)
////    {
////        transition
////        {
////            if(libCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 60.f) ||
////                    libCodeRelease.between(theBallModel.estimate.position.y(), -60.f, -30.f) )
////            {
////                if( libCodeRelease.between(theBallModel.estimate.position.x(), 100.f, 170.f) &&
////                        std::abs(libCodeRelease.getAngleToTeammate()) < Angle::fromDegrees(1.f) )
////                    goto kick;
////            }
////            std::cerr << theBallModel.estimate.position.angle() << std::endl;
////            if (!libCodeRelease.between(fabs(theBallModel.estimate.position.angle()), Angle::fromDegrees(0), Angle::fromDegrees(90) ))
////                goto turnToBall;
////            if (theBallModel.estimate.position.norm() > 500)
////                goto walkToBall;
////            if(libCodeRelease.timeSinceBallWasSeen() > 1000)
////                goto beStationary;
////        }
////        action
////        {
////            theHeadControlMode = HeadControl::lookAtBall;
////            if( theBallModel.estimate.position.y() >= .0f )
////                WalkToTarget(Pose2f(80.f, 80.f, 50.f),
////                             Pose2f(libCodeRelease.getAngleToTeammate(), theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 50.f));
////            else
////                WalkToTarget(Pose2f(80.f, 80.f, 50.f),
////                             Pose2f(libCodeRelease.getAngleToTeammate(), theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() + 50.f));
////        }
////    }

////    state(turnToBall)
////    {
////        transition
////        {


////            if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(5.f)){
////                if(theBallModel.estimate.position.norm() < 300.f)
////                    goto alignToOtherRobot;
////                goto walkToBall;
////            }

////        }
////        action
////        {
////            theHeadControlMode = HeadControl::lookForward;
////            WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
////        }
////    }

////    state(walkToBall)
////    {
////        transition
////        {
////            if (((Vector2f)(libCodeRelease.getTeammatePosition() - theBallModel.estimate.position)).norm()
////                    < theBallModel.estimate.position.norm())
////                goto beStationary;
////            if(theBallModel.estimate.position.norm() < 300.f)
////                goto alignToOtherRobot;
////            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(5.f))
////                goto turnToBall;
////            if(libCodeRelease.timeSinceBallWasSeen() > 1000)
////                goto beStationary;
////        }
////        action
////        {
////            WalkToTarget(Pose2f(.2f, SPQR::WALKING_VELOCITY_X, 20.f), theBallModel.estimate.position);
////        }
////    }

////    state(kick)
////    {
////        transition
////        {
////            if (action_done || state_time > 3000)
////                goto beStationary;
////        }
////        action
////        {
////            theHeadControlMode = HeadControl::lookAtBall;
////            Kicks("fastForwardKick");
////        }
////    }
////}






option(PassBall)
{

        common_transition
        {
    //        // se la palla è più lontana di un metro e mezzo fermati
    //        if (((Vector2f)(libCodeRelease.getTeammatePosition() - theBallModel.estimate.position)).norm()
    //                > theBallModel.estimate.position.norm())
    //            goto beStationary;
            cout<< "diritto " << libCodeRelease.getTeammatePosition() << endl;
        }


    initial_state(start)
    {
        transition
        {
            if(libCodeRelease.getTeammatePosition().norm() != 0.f){

                if( libCodeRelease.timeSinceBallWasSeen() > 3000 )
                    goto lookAround;
                else goto walkToBall;
            }


        }
        action
        {
            if(libCodeRelease.getTeammatePosition().norm() == 0.f){
                theHeadControlMode = HeadControl::lookLeftAndRight;
            }
            else
                theHeadControlMode = HeadControl::lookAtBall;

            Stand();
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
                goto alignToOtherRobot;

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

    state(alignToOtherRobot)
    {
        transition
        {

            if(std::abs(libCodeRelease.getAngleToTeammate()) < Angle::fromDegrees(4.f))
            {
                if (std::abs(theBallModel.estimate.position.y()) < 150.f && std::abs(theBallModel.estimate.position.x()) < 250.f)
                    goto alignBehindBall;

            }

            if (!libCodeRelease.between(fabs(theBallModel.estimate.position.angle()), Angle::fromDegrees(0), Angle::fromDegrees(90) ))
                goto walkToBall;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            WalkToTarget(Pose2f(0.4f, 10.f, 30.f), Pose2f(libCodeRelease.getAngleToTeammate(), theBallModel.estimate.position.x() - 300.f, theBallModel.estimate.position.y()));
        }
    }


    state(alignBehindBall)
    {
        transition
        {
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 60.f) ||
                    libCodeRelease.between(theBallModel.estimate.position.y(), -60.f, -30.f) )
            {
                if( libCodeRelease.between(theBallModel.estimate.position.x(), 100.f, 170.f) &&
                        std::abs(libCodeRelease.getAngleToTeammate()) < Angle::fromDegrees(1.f) )
                    goto kick;
            }
            std::cerr << theBallModel.estimate.position.angle() << std::endl;
            if ((!libCodeRelease.between(fabs(theBallModel.estimate.position.angle()), Angle::fromDegrees(0), Angle::fromDegrees(90) )) ||
                    theBallModel.estimate.position.norm() > 500)
                goto walkToBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            if( theBallModel.estimate.position.y() >= .0f )
                WalkToTarget(Pose2f(80.f, 80.f, 50.f),
                             Pose2f(libCodeRelease.getAngleToTeammate(), theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 50.f));
            else
                WalkToTarget(Pose2f(80.f, 80.f, 50.f),
                             Pose2f(libCodeRelease.getAngleToTeammate(), theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() + 50.f));
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


    state(beStationary)
    {
        transition
        {
            if (((Vector2f)(libCodeRelease.getTeammatePosition() - theBallModel.estimate.position)).norm()
                    > theBallModel.estimate.position.norm())
                goto walkToBall;
            if (std::abs(libCodeRelease.getAngleToTeammate()) > Angle::fromDegrees(4.f))
                goto alignToOtherRobot;

        }
        action
        {
            if(theFrameInfo.time - theBallModel.timeWhenLastSeen < 1000)
                theHeadControlMode = HeadControl::lookAtBall;
            else
                theHeadControlMode = HeadControl::lookLeftAndRight/*UpAndDown*/;
            //~
            Stand();
        }
    }
}

//option(PassBall){

//    initial_state(init)
//    {
//        transition
//        {



//        }
//        action
//        {
//            //            if( theSpqrDWKcombiner.estimated_ball_global.norm() > 1000.f || theBallModel.estimate.position.norm() > 1000.f)
//            //            {
//            //                //                cout << state_time%2000 << "  " << ((state_time%4000 < 2000 ) ? "RightLeft": "landmarks") << endl;
//            //                if( theSpqrDWKcombiner.estimated_ball_global.norm() < 2500.f || theBallModel.estimate.position.norm() < 2500.f)
//            //                    theHeadControlMode = HeadControl::lookLeftAndRightForStriker;
//            //                else
//            //                    theHeadControlMode = HeadControl::lookAtLandmark;
//            //            }
//            //            else
//            {
//                theHeadControlMode = HeadControl::lookLeftAndRight;
//            }

//            if(std::abs(libCodeRelease.getAngleToTeammate()) > Angle::fromDegrees(10.f)){
//                WalkToTarget(Pose2f(0.4f, 10.f, 30.f), Pose2f(libCodeRelease.getAngleToTeammate(), 0, 0));
//                cout<< "angolo " << libCodeRelease.getAngleToTeammate() << endl;
//            }
//            else
//            {
//                WalkToTarget( Pose2f(.0f, SPQR::WALKING_VELOCITY_X, 0.f), Pose2f(libCodeRelease.getTeammatePosition()) );
//                cout<< "diritto " << libCodeRelease.getTeammatePosition() << endl;
//            }

//        }
//    }
//}
