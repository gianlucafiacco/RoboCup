
option(Guard)
{
    initial_state(start)
    {
        transition
        {
            goto lookAround;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            Stand();
        }
    }


    state(turnToOpponentGoal)
    {
        transition
        {
            if(std::abs(theRobotPoseSpqrFiltered.theta) < Angle::fromDegrees(5.f))
                goto stand;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            if( theRobotPoseSpqrFiltered.theta > .0f )
                WalkAtSpeedPercentage(Pose2f(-.6f, 0.f, 0.f));
            else
                WalkAtSpeedPercentage(Pose2f(.6f, 0.f, 0.f));
        }
    }

    state(lookAround)
    {
        transition
        {
            if( state_time > 10000 )
                goto turnToOpponentGoal;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            WalkAtSpeedPercentage(Pose2f(-.6f, 0.f, 0.f));
        }
    }

    state(stand)
    {
        transition
        {
            if(std::abs(theRobotPoseSpqrFiltered.theta) > Angle::fromDegrees(5.f))
                goto turnToOpponentGoal;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            Stand();
        }
    }
}
