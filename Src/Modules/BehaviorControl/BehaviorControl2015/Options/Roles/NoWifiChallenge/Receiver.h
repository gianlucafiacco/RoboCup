
option(Receiver)
{

//    common_transition {goto pointCloseRight;}

    initial_state(left)
    {
        transition
        {
            if (theNoWifiReceivedPacket.available)
            {    
                if (theNoWifiReceivedPacket.packet.header.type == 1)
                {
                    int16_t x = theNoWifiReceivedPacket.packet.payload.location.x;
                    int16_t y = theNoWifiReceivedPacket.packet.payload.location.y;
                    bool rightSpot = true;
                    x -= 4500;
                    y -= 3000;
                    if (y <= 0)
                    {
                        rightSpot = !rightSpot;
                        y = -y;
                    }

                    if (y < 800)
                    {
                        if (x <= -3900)
                        {
                            if (rightSpot)
                                goto pointCloseLeft;
                            else
                                goto pointCloseRight;
                        }
                        else if (x >= 0)
                        {
                            if (rightSpot)
                                goto pointCenterLeft;
                            else
                                goto pointCenterRight;
                        }
                        else
                        {
                            if (rightSpot)
                                goto pointCenterLeftDown;
                            else
                                goto pointCenterRightDown;
                        }
                    }
                    else
                    {
                        if (x <= -3200)
                        {
                            if (rightSpot)
                                goto pointExtremeLeft;
                            else
                                goto pointExtremeRight;
                        }
                        else if (x >= 0)
                        {
                            if (rightSpot)
                                goto pointModerateLeft;
                            else
                                goto pointModerateRight;
                        }
                        else
                        {
                            if (rightSpot)
                                goto pointLeft;
                            else
                                goto pointRight;
                        }
                    }


                }
            }

        }

        action
        {
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(90);
            theHeadMotionRequest.tilt = Angle::fromDegrees(0);  //25
            theHeadMotionRequest.speed = pi/5; //fromDegrees(200);


        }
    }

    state(pointCenterLeft)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingCenterLeft;
        }
    }

    state(pointCenterLeftDown)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingCenterLeftDown;
        }
    }

    state(pointCenterRight)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingCenterRight;
        }
    }

    state(pointCenterRightDown)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingCenterRightDown;
        }
    }

    state(pointCloseLeft)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingCloseLeft;
        }
    }

    state(pointCloseRight)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingCloseRight;
        }
    }

    state(pointExtremeLeft)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingExtremeLeft;
        }
    }

    state(pointExtremeRight)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingExtremeRight;
        }
    }

    state(pointLeft)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingLeft;
        }
    }

    state(pointModerateLeft)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingModerateLeft;
        }
    }

    state(pointModerateRight)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingModerateRight;
        }
    }

    state(pointRight)
    {
        transition
        {
            if (state_time > 1000)
		goto left;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            theMotionRequest.motion = MotionRequest::specialAction;
            theMotionRequest.specialActionRequest.mirror = false;
            theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::pointingRight;
        }
    }

};
