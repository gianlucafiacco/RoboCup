
option(Walker)
{
    initial_state(walk)
    {
        action
        {
            WalkAtSpeed( Pose2f(0.f, 100.f, 0.f) );  //velocity in mm/s (theta, x, y)
        }
    }

}

