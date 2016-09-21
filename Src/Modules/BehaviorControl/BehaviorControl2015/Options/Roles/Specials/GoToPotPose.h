
option(GoToPotPose)
{
    initial_state(start)
    {
        action
        {
            WalkAtSpeedPercentage(Pose2f(0.f,theSpqrDWKcombiner.vel_avg.x(),theSpqrDWKcombiner.vel_avg.y()));
        }
    }
}

