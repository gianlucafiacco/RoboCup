/** behavior for the ready state */

bool leftEntering=false;

option(ReadyState)
{
    initial_state(start)
    {
        transition
        {
            if(state_time > 300)
                goto wait;
        }
        action
        {

            if(theRobotPoseSpqrFiltered.y > 0.0) leftEntering = true;

            theHeadControlMode = HeadControl::lookForward;
            Stand();
        }
    }

    state(wait)
    {
        transition
        {
            if( theRobotInfo.number == 1 ) goto gotoPosition;
            else if( theRobotInfo.number == 5 ) goto gotoPosition;
            else if( theRobotInfo.number == 2 && state_time > 1000 ) goto gotoPosition;
            else if( state_time > 5000 ) goto gotoPosition;
            else goto gotoPosition;

        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            Stand();
        }
    }

    target_state(gotoPosition)
    {
        transition
        {

        }
        action
        {
            if( theRobotInfo.number == 1 ) theHeadControlMode = HeadControl::lookLeftAndRightUpAndDown;
            theHeadControlMode = HeadControl::lookLeftAndRight;

            if(theRobotInfo.number == 1)
                Goalie();
            else
            {
                if( theGameInfo.kickOffTeam != theOwnTeamInfo.teamColor )
                {
                    Pose2f readyPose(libCodeRelease.getReadyPose(false, (ContextCoordination::SpqrRole)(theRobotInfo.number) ));
                    GetIntoReadyPosition(Pose2f( .6f, 70.f, 20.f), readyPose.translation );
                }
                else
                {
                    Pose2f readyPose(libCodeRelease.getReadyPose(true, (ContextCoordination::SpqrRole)(theRobotInfo.number) ));
                    GetIntoReadyPosition(Pose2f( .6f, 70.f, 20.f), readyPose.translation );
                }
            }
        }
    }
}
