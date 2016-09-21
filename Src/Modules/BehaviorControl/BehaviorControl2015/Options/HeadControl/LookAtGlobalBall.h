option(lookAtGlobalBall)
{
    initial_state(lookAtGlobalBall)
    {
        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::targetMode;
            if(theSpqrDWKcombiner.current_context == SpqrDWKcombiner::playing)
            {
                theHeadMotionRequest.target.x() = theSpqrDWKcombiner.estimated_ball_relative.x();
                theHeadMotionRequest.target.y() = theSpqrDWKcombiner.estimated_ball_relative.y();
            }
            else
            {
                theHeadMotionRequest.target.x() = theSpqrDWKcombiner.estimated_ball_relative.x();
                theHeadMotionRequest.target.y() = theSpqrDWKcombiner.estimated_ball_relative.y();
            }
            
            theHeadMotionRequest.target.z() = 35;
            theHeadMotionRequest.speed = 1;
        }
    }

}
