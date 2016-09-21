//Option for the striker
option(lookLeftAndRightForStriker)
{
    initial_state(left)
    {
        transition
        {
            if (state_time > 2500) goto right;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(55);
            theHeadMotionRequest.tilt = Angle::fromDegrees(25);
            theHeadMotionRequest.speed = pi/2; //fromDegrees(200);
        }
    }

    state(right)
    {
        transition
        {
            if (state_time > 2500) goto left;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-55);
            theHeadMotionRequest.tilt = Angle::fromDegrees(25);
            theHeadMotionRequest.speed = pi/2; //fromDegrees(200);
        }
    }
};
