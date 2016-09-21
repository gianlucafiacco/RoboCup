
option(lookLeftAndRightUpAndDown)
{
    initial_state(left)
    {
        transition
        {
            if (state_time > 1300) goto right;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(45);
            theHeadMotionRequest.tilt = Angle::fromDegrees(-18);
            theHeadMotionRequest.speed = pi/4; //fromDegrees(200);
        }
    }

    state(right)
    {
        transition
        {
            if (state_time > 1300) goto rightDown;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-45);
            theHeadMotionRequest.tilt = Angle::fromDegrees(-18);
            theHeadMotionRequest.speed = pi/4; //fromDegrees(200);
        }
    }
    
    state(rightDown)
    {
        transition
        {
            if (state_time > 1300) goto leftDown;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-45);
            theHeadMotionRequest.tilt = Angle::fromDegrees(30);
            theHeadMotionRequest.speed = pi/4; //fromDegrees(200);
        }
    }

    state(leftDown)
    {
        transition
        {
            if (state_time > 1300) goto left;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(45);
            theHeadMotionRequest.tilt = Angle::fromDegrees(30);
            theHeadMotionRequest.speed = pi/4; //fromDegrees(200);
        }
    }
};
