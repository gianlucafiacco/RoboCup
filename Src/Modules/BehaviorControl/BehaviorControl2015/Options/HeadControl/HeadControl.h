option(HeadControl)
{
    common_transition
    {
        if(!theGroundContactState.contact && theGameInfo.state != STATE_INITIAL)
            goto lookForward;

        switch(theHeadControlMode)
        {
        case HeadControl::off: goto off;
        case HeadControl::lookForward: goto lookForward;
        case HeadControl::lookForwardGoalie: goto lookForwardGoalie;
        case HeadControl::lookAtBall: goto lookAtBall;
        case HeadControl::lookAtGlobalBall: goto lookAtGlobalBall;
        case HeadControl::lookAtLandmark: goto lookAtLandmark;
        case HeadControl::lookLeftAndRight: goto lookLeftAndRight;
        case HeadControl::lookLeftAndRightReduced: goto lookLeftAndRightReduced;
        case HeadControl::lookLeftAndRightUpAndDown: goto lookLeftAndRightUpAndDown;
        case HeadControl::lookAroundForLocalizer: goto lookAroundForLocalizer;
        case HeadControl::lookLeftAndRightForStriker: goto lookLeftAndRightForStriker;
        default: goto none;
        }
    }

    initial_state(none) {}
    state(off) {action SetHeadPanTilt(JointAngles::off, JointAngles::off, 0.f);}
    state(lookForward) {action LookForward();}
    state(lookForwardGoalie) {action LookForward(.1f);}
    state(lookAtBall) {action lookAtBall();}
    state(lookAtGlobalBall) {action lookAtGlobalBall();}
    state(lookAtLandmark) {action lookAtLandmark();}
    state(lookLeftAndRight) {action lookLeftAndRight();}
    state(lookLeftAndRightReduced) {action lookLeftAndRightReduced();}
    state(lookLeftAndRightUpAndDown) {action lookLeftAndRightUpAndDown();}
    state(lookAroundForLocalizer) {action lookAroundForLocalizer();}
    state(lookLeftAndRightForStriker) {action lookLeftAndRightForStriker();}

}

struct HeadControl
{
    ENUM(Mode,
    {,
     none,
     off,
     lookForward,
     lookForwardGoalie,
     lookAtBall,
     lookAtGlobalBall,
     lookAtLandmark,
     lookLeftAndRight,
     lookLeftAndRightReduced,
     lookLeftAndRightUpAndDown,
     lookAroundForLocalizer,
     lookLeftAndRightForStriker,
         });
};

HeadControl::Mode theHeadControlMode = HeadControl::Mode::none; /**< The head control mode executed by the option HeadControl. */
