option(KickToTarget, (const Pose2f&) target)
{
    initial_state(start)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() < 300.f)
                goto alignToTarget;
            else if(theBallModel.estimate.position.norm() > 300.f)
                goto turnToBall;
        }
        action
        {
            Stand();
        }
    }

    state(turnToBall)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() < 300.f)
                goto alignToTarget;

            if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(5.f))
                goto walkToBall;

        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
        }
    }

    state(walkToBall)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() < 300.f)
                goto alignToTarget;
        }
        action
        {
            WalkToTarget(Pose2f(.2f, 70.f, 20.f), theBallModel.estimate.position);
        }
    }

    state(alignToTarget)
    {
        transition
        {

            if(std::abs(libCodeRelease.angleToTarget(target.translation.x(), target.translation.y())) < Angle::fromDegrees(5.f) &&
                    std::abs(theBallModel.estimate.position.y()) < 100.f)
                goto alignBehindBall;
        }
        action
        {
            WalkToTarget(Pose2f(0.4f, 10.f, 30.f),
                         Pose2f(libCodeRelease.angleToTarget(target.translation.x(), target.translation.y()),
                                theBallModel.estimate.position.x() - 300.f, theBallModel.estimate.position.y()));
        }
    }

    state(alignBehindBall)
    {
        transition
        {

            if(libCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 60.f) ||
                    libCodeRelease.between(theBallModel.estimate.position.y(), -60.f, -30.f) )
            {
                if( libCodeRelease.between(theBallModel.estimate.position.x(), 100.f, 170.f) &&
                        std::abs(libCodeRelease.angleToTarget(target.translation.x(), target.translation.y())) < Angle::fromDegrees(3.f) )
                    goto kick;
            }
        }
        action
        {
            if( theBallModel.estimate.position.y() >= .0f )
                WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                             Pose2f(libCodeRelease.angleToTarget(target.translation.x(), target.translation.y()),
                                    theBallModel.estimate.position.x() - 150.f,
                                    theBallModel.estimate.position.y() - 50.f));
            else
                WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                             Pose2f(libCodeRelease.angleToTarget(target.translation.x(), target.translation.y()),
                                    theBallModel.estimate.position.x() - 150.f,
                                    theBallModel.estimate.position.y() + 50.f));
        }
    }

    target_state(kick)
    {
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
#ifndef CORNER_KICK
            Kicks("fastForwardKick");
#else
            Kicks("cornerKick");
#endif

        }
    }
}
