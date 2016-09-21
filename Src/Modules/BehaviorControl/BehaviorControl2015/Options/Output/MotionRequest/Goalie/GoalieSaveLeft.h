
option(GoalieSaveLeft)
{
  initial_state(goalieSaveLeft)
  {
    transition
    {
    }
    action
    {
        theHeadControlMode = HeadControl::lookForward;

        theMotionRequest.motion = MotionRequest::specialAction;
        theMotionRequest.specialActionRequest.mirror = false;
        theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::keeperJumpLeft;
    }
  }
}

