

option(GotoPose, (const Pose2f&) target)
{
    initial_state(start)
    {
        transition
        {
            if( libCodeRelease.norm(target.translation.x(), target.translation.y()) > 250)
				goto turnToReadyPosition;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
			Stand();
        }
    }

	state(turnToReadyPosition)
	{
		transition
        {
            if(std::abs(target.translation.angle()) < Angle::fromDegrees(5.f))
				goto walkToReadyPosition;
		}
		action
		{
			theHeadControlMode = HeadControl::lookForward;
            WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(target.translation.angle(), 0.f, 0.f));
		}
	}

	state(walkToReadyPosition)
	{
		transition
        {
            if( libCodeRelease.isValueBalanced(theRobotPose.translation.x(),
                                               libCodeRelease.rel2Glob(target.translation.x(), target.translation.y()).translation.x(), 200) &&
                    libCodeRelease.isValueBalanced(theRobotPose.translation.y(),
                                                   libCodeRelease.rel2Glob(target.translation.x(), target.translation.y()).translation.y(), 200) )
                goto getReady;

            if( std::abs(target.translation.angle()) > Angle::fromDegrees(5.f) )
                goto turnToReadyPosition;
		}
		action
		{
//			theHeadControlMode = HeadControl::lookAtLandmark;
            SetHeadTargetOnGround(Vector3f(target.translation.x(), target.translation.y(), 10.f));
            WalkToTarget( Pose2f(.2f, 40.f, 30.f), target.translation);
		}
	}

    state(getReady)
    {
        transition
        {
            if( libCodeRelease.between(theRobotPose.rotation,
                                       Angle::fromDegrees(target.rotation - 10.f) ,
                                       Angle::fromDegrees(target.rotation + 10.f)  ) )
            {
                    if( libCodeRelease.norm(target.translation.x(), target.translation.y()) < 250)
                    goto start;
            }

            if( libCodeRelease.norm(target.translation.x(), target.translation.y()) > 250)
                goto start;

        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            WalkAtSpeedPercentage(Pose2f(.6f, 0.f, 0.f));
        }
    }

}
