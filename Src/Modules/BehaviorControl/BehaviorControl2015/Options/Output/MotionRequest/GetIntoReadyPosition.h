
//#define DEBUG_READY_POSITIONING

option(GetIntoReadyPosition, (const Pose2f&) speed, (const Pose2f&) target)
{
    initial_state(start)
    {
        transition
        {
#ifdef DEBUG_READY_POSITIONING
            STATE("start");
#endif
			if( libCodeRelease.norm(target.translation.x(), target.translation.y()) > 250)
                goto walkToReadyPosition;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
			Stand();
        }
    }

	state(walkToReadyPosition)
	{
		transition
		{
#ifdef DEBUG_READY_POSITIONING
            STATE("walkToReadyPosition");
#endif

			if( libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.x,
                                               libCodeRelease.rel2Glob(target.translation.x(), target.translation.y()).translation.x(), 400) &&
					libCodeRelease.isValueBalanced(theRobotPoseSpqrFiltered.y,
                                                   libCodeRelease.rel2Glob(target.translation.x(), target.translation.y()).translation.y(), 400) )
                goto getReady;
		}
		action
		{
            theHeadControlMode = HeadControl::lookLeftAndRightUpAndDown;
            if(std::abs(target.translation.angle()) > Angle::fromDegrees(10.f))
                WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(target.translation.angle(), 0.f, 0.f));
            else
                WalkToTarget( Pose2f(0.f, SPQR::WALKING_VELOCITY_X, 0.f), target );
		}
	}

    state(getReady)
    {
        transition
        {
#ifdef DEBUG_READY_POSITIONING
            STATE("getReady");
#endif
            if( libCodeRelease.between(theRobotPoseSpqrFiltered.theta,
                                                   libCodeRelease.angleToGoal - Angle::fromDegrees(10.f) ,
                                                   libCodeRelease.angleToGoal + Angle::fromDegrees( 10.f)  ) )
                goto start;

        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRightUpAndDown;

			if( theRobotPoseSpqrFiltered.theta > .0f )
                WalkAtSpeedPercentage(Pose2f(-50.f, 0.f, 0.f));
			else
                WalkAtSpeedPercentage(Pose2f(50.f, 0.f, 0.f));
        }
    }

}
