//Stop and scan. Scan and stop.
option(lookLeftAndRight)
{
    initial_state(zero)
    {
		transition
        {
            if (state_time > 600) goto one;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(0);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
        }
    }

    state(one)
    {
		transition
        {
            if (state_time > 600) goto two;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(22);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
        }
    }

    state(two)
    {
        transition
        {
            if (state_time > 600) goto three;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(60);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
        }
    }

    state(three)
    {
        transition
        {
            if (state_time > 600) goto four;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(22);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
        }
    }

    state(four)
    {
        transition
        {
            if (state_time > 600) goto five;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(0);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
        }
    }

    state(five)
    {
        transition
        {
            if (state_time > 600) goto six;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-22);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
        }
    }

    state(six)
    {
        transition
        {
            if (state_time > 600) goto seven;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-60);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
        }
    }
    state(seven)
    {
        transition
        {
            if (state_time > 600) goto zero;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-22);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
        }
    }
};
