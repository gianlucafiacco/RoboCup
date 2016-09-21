
option(lookAtBall)
{
    initial_state(lookAtBall)
    {
        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::targetMode;
            theHeadMotionRequest.target.x() = theBallModel.estimate.position.x();
            theHeadMotionRequest.target.y() = theBallModel.estimate.position.y();
            theHeadMotionRequest.target.z() = 35;
            theHeadMotionRequest.speed = 1;
        }
    }

}
