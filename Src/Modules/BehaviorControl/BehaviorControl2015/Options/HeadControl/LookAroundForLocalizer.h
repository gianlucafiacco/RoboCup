
option(lookAroundForLocalizer)
{
    initial_state(left)
    {
        transition
        {
            if (state_time > 1600) goto leftDown;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(60);
            theHeadMotionRequest.tilt = Angle::fromDegrees(-18);
            theHeadMotionRequest.speed = pi/4; //fromDegrees(200);
        }
    }

    state(leftDown) //TODO Vincenzo
    {
        transition
        {
            if (state_time > 1600) goto rightDown;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(60);
            theHeadMotionRequest.tilt = Angle::fromDegrees(23);
            theHeadMotionRequest.speed = pi/4; //fromDegrees(200);
        }
    }

    state(rightDown) //TODO Vincenzo
    {
        transition
        {
            if (state_time > 1800) goto right;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-60);
            theHeadMotionRequest.tilt = Angle::fromDegrees(23);
            theHeadMotionRequest.speed = pi/4; //fromDegrees(200);
        }
    }

    state(right)
    {
        transition
        {
            if (state_time > 1600) goto extremeRight;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-60);
            theHeadMotionRequest.tilt = Angle::fromDegrees(-18);
            theHeadMotionRequest.speed = pi/4; //fromDegrees(200);
        }
    }

    state(extremeRight)
    {
        transition
        {
            if (state_time > 3000) goto extremeLeft;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-80);
            theHeadMotionRequest.tilt = Angle::fromDegrees(-18);
            theHeadMotionRequest.speed = pi/4; //fromDegrees(200);
        }
    }

    state(extremeLeft)
    {
        transition
        {
            if (state_time > 4000) goto left;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(80);
            theHeadMotionRequest.tilt = Angle::fromDegrees(-18);
            theHeadMotionRequest.speed = pi/4; //fromDegrees(200);
        }
    }
};
