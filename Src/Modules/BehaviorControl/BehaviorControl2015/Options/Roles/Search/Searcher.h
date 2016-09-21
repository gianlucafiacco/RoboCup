/**
 * Searcher: used in case of fixed positioning RoboCup 2016. If the ball is not seen
 * TODO rewrite if the ball becomes stable (maybe pre 2016?)!!!
 */

option(Searcher, (const int) searcherNumber)
{
#ifdef NO_COORDINATED_ROLES
    float x;
    float y;

    if(searcherNumber == 1){
        x = SPQR::GOALIE_BASE_POSITION_X;
        y = SPQR::GOALIE_BASE_POSITION_X;
    }
    if(searcherNumber == 2){
        x = SPQR::DEFENDER_DEFAULT_POSITION_X;
        y = SPQR::DEFENDER_DEFAULT_POSITION_Y;
    }
    if(searcherNumber == 3){
        x = SPQR::SUPPORTER_DEFAULT_POSITION_X;
        y = SPQR::SUPPORTER_DEFAULT_POSITION_Y;
    }
    if(searcherNumber == 4){
        x = SPQR::JOLLY_DEFAULT_POSITION_X;
        y = SPQR::JOLLY_DEFAULT_POSITION_Y;
    }
    if(searcherNumber == 5){
        x = 0;
        y = 0;
    }

    initial_state(start)
    {
        transition
        {
            if((libCodeRelease.ballIsInGame() || state_time > 10000))
                goto turnToCentroid;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            Stand();
        }
    }

    state(turnToCentroid)
    {
        transition
        {
            if(std::abs(libCodeRelease.angleToTarget(x,y)) < Angle::fromDegrees(5.f))
                goto walkToCentroid;
        }
        action
        {

            theHeadControlMode = HeadControl::lookLeftAndRight;
            WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(libCodeRelease.angleToTarget(x,y), 0.f, 0.f));
        }
    }

    state(walkToCentroid)
    {
        transition
        {
            if(std::abs(libCodeRelease.angleToTarget(x,y)) > Angle::fromDegrees(5.f))
                goto turnToCentroid;

            if( libCodeRelease.norm(theRobotPoseSpqrFiltered.x - x,
                                    theRobotPoseSpqrFiltered.y - y) < 300 )
                goto turnToOpponentGoal;

        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            WalkToTarget( Pose2f(.2f, SPQR::WALKING_VELOCITY_X, 30.f), libCodeRelease.glob2Rel(x,y));
        }
    }

    state(lookAround)
    {
        transition
        {
            if( libCodeRelease.norm(theRobotPoseSpqrFiltered.x - x,
                                    theRobotPoseSpqrFiltered.y - y) < 300 )
                goto walkToCentroid;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
        }
    }

    state(turnToOpponentGoal)
    {
        transition
        {
            if( libCodeRelease.norm(theRobotPoseSpqrFiltered.x - x,
                                    theRobotPoseSpqrFiltered.y - y) > 800)
                goto turnToCentroid;

            if( libCodeRelease.between(theRobotPoseSpqrFiltered.theta,
                                       libCodeRelease.angleToGoal + Angle::fromDegrees(-10.f) ,
                                       libCodeRelease.angleToGoal + Angle::fromDegrees( 10.f)  ) )
                goto lookAround;
        }
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight/*UpAndDown*/;
            WalkToTarget(Pose2f(50.f, 0.f, 0.f), Pose2f(libCodeRelease.angleToGoal, 0.f, 0.f));
        }
    }
#else
    initial_state(lookAround)
    {
        transition
        {}
        action
        {
            theHeadControlMode = HeadControl::lookLeftAndRight;
            if( libCodeRelease.ballOutOnLeft )
                WalkAtSpeedPercentage(Pose2f(-.8f, 0.f, 0.f));
            else
                WalkAtSpeedPercentage(Pose2f(.8f, 0.f, 0.f));
        }
    }
#endif
}
