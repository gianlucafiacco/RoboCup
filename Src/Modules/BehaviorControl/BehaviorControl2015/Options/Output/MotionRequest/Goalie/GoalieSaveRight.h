
option(GoalieSaveRight)
{
  initial_state(goalieSaveRight)
  {
    transition
    {
    }
    action
    {
        theHeadControlMode = HeadControl::lookForward;

        theMotionRequest.motion = MotionRequest::specialAction;
        theMotionRequest.specialActionRequest.mirror = false;
        theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::keeperJumpRight;
    }
  }
}

