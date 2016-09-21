
option(GoalieSitAndSaveLeft)
{
  initial_state(goalieSitAndSaveLeft)
  {
    action
    {
        theHeadControlMode = HeadControl::lookForward;

        theMotionRequest.motion = MotionRequest::specialAction;
        theMotionRequest.specialActionRequest.mirror = false;
        theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::keeperSitAndJumpLeft;
    }
  }
}

