
option(GoalieSitAndSaveRight)
{
  initial_state(goalieSitAndSaveRight)
  {
    action
    {
        theHeadControlMode = HeadControl::lookForward;

        theMotionRequest.motion = MotionRequest::specialAction;
        theMotionRequest.specialActionRequest.mirror = true;
        theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::keeperSitAndJumpLeft;
    }
  }
}

