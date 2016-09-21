option(Throwin , (const int) throwinNumber)
{
    initial_state(start)
    {
        transition
        {

            if(std::abs(libCodeRelease.angleToTarget(theSpqrDWKcombiner.unexplored_clusters_centroids.at(throwinNumber-1).x(),
                                                     theSpqrDWKcombiner.unexplored_clusters_centroids.at(throwinNumber-1).y())) > Angle::fromDegrees(7.f))
                goto turnToThrowInPosition;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            Stand();
        }
    }

    state(turnToThrowInPosition)
    {
        transition
        {
            if(std::abs(libCodeRelease.angleToTarget(theSpqrDWKcombiner.unexplored_clusters_centroids.at(throwinNumber-1).x(),
                                                     theSpqrDWKcombiner.unexplored_clusters_centroids.at(throwinNumber-1).y())) < Angle::fromDegrees(7.f))
                goto start;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(libCodeRelease.angleToTarget(theSpqrDWKcombiner.unexplored_clusters_centroids.at(throwinNumber-1).x(),
                                                                                     theSpqrDWKcombiner.unexplored_clusters_centroids.at(throwinNumber-1).y()), 0.f, 0.f));
        }
    }

}
