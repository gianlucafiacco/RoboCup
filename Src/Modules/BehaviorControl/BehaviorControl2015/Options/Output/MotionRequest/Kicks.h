option(Kicks, (const std::string) kick)
{
    initial_state(Kicks)
    {
        transition{
            if(kick == "forwardKick")
                goto forwardKick;
            else if(kick == "fastForwardKick")
                goto fastForwardKick;
            else if(kick == "veryFastForwardKick")
                goto veryFastForwardKick;
            else if(kick == "sideKick")
                goto sideKick;
            else if(kick == "backKick")
                goto backKick;
            else if(kick == "extSideKick")
                goto extSideKick;
            else if(kick == "sidewardKick")
                goto sidewardKick;
            else if(kick == "cornerKick")
                goto cornerKick;
#ifdef DEMOKICK
            else if(kick == "realBallKickWithStand")
                goto realBallKickWithStand;
#endif
        }
    }

    state(forwardKick)
    {
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;

            theMotionRequest.kickRequest.dynamical=true;

            theMotionRequest.kickRequest.dynPoints.clear();

            if (theMotionRequest.kickRequest.mirror)
            {
                if(theBallModel.estimate.position.y()<90)
                {
//                    DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
//                                                    -90,
//                                                    -190});
//                    theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                }

                else{
//                    DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
//                                                    -theBallModel.estimate.position.y(),
//                                                    -190});
//                    theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                }
            }

            else
            {
                if(theBallModel.estimate.position.y()>-90)
                {
//                    DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
//                                                    -90,
//                                                    -190});
//                    theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                }

                else
                {
//                    DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
//                                                    theBallModel.estimate.position.y(),
//                                                    -190});
//                    theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                }
            }

            theMotionRequest.kickRequest.kickMotionType = KickRequest::kickForward;
        }
    }


    state(fastForwardKick)
    {
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;

            theMotionRequest.kickRequest.dynamical=true;

            theMotionRequest.kickRequest.dynPoints.clear();

            if (theMotionRequest.kickRequest.mirror)
            {
                if(theBallModel.estimate.position.y()<90)
                {
                    DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
                                                    -90,
                                                    -190});
                    theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                }

                else{
                    DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
                                                    -theBallModel.estimate.position.y(),
                                                    -190});
                    theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                }
            }

            else
            {
                if(theBallModel.estimate.position.y()>-90)
                {
                    DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
                                                    -90,
                                                    -190});
                    theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                }

                else
                {
                    DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
                                                    theBallModel.estimate.position.y(),
                                                    -190});
                    theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                }
            }

            theMotionRequest.kickRequest.kickMotionType = KickRequest::fastKickForward;
        }
    }

    state(veryFastForwardKick)
    {
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;

            //theMotionRequest.kickRequest.dynamical=true;

            theMotionRequest.kickRequest.dynPoints.clear();
            theMotionRequest.kickRequest.kickMotionType = KickRequest::veryFastKickForward;
        }
    }


    state(cornerKick)
    {
        action
        {
            theMotionRequest.kickRequest.dynamical=false;
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::cornerKick;

        }
    }

    state(extSideKick)
    {
        action
        {
            theMotionRequest.kickRequest.dynamical=false;
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::extSideKick;
        }
    }

    state(backKick)
    {
        action
        {
            theMotionRequest.kickRequest.dynamical=false;
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() >= .0f ?  false : true;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::backKick;
        }
    }

    state(sideKick)
    {
        action
        {
            theMotionRequest.kickRequest.dynamical=false;
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = libCodeRelease.glob2Rel(SPQR::FIELD_DIMENSION_X, .0f).translation.y() < .0f ?  true : false;

            theMotionRequest.kickRequest.kickMotionType = KickRequest::sideKick;
        }
    }

    state(sidewardKick)
    {
        action
        {
            theMotionRequest.kickRequest.dynamical=false;
            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::kickSideward;
        }
    }
    
#ifdef DEMOKICK
    state(realBallKickWithStand)
    {
        action
        {
            theMotionRequest.kickRequest.dynamical=false;
            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::realBallKickWithStand;
        }
    }
#endif
}
