
#define SCALE 0.2

option(GoalieLearner)
{
    initial_state(start)
    {
        transition
        {
            goto mainLoop;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            Stand();
        }
    }

    state(mainLoop)
    {
        transition
        {
            //Technical goalie computation BEGIN    
			float technicalDiveTime;
            if(!libCodeRelease.diveBool)
			{
                technicalDiveTime = theDiveHandle.diveTime*1.5;
#ifdef SPQR_DEBUG_GOALIE
                    std::cout<<"Bool: "<<libCodeRelease.diveBool<<" dive time: "<<theDiveHandle.diveTime<<" Tech dive time: "<<technicalDiveTime<<std::endl;
#endif
            }
			else
			{
                technicalDiveTime = theDiveHandle.diveTime*SCALE;
#ifdef SPQR_DEBUG_GOALIE
                    std::cout<<"Bool: "<<libCodeRelease.diveBool<<" dive time: "<<theDiveHandle.diveTime<<" Tech dive time: "<<technicalDiveTime<<std::endl;
#endif
			}
			//Technical goalie computation END

            if(std::abs(theDiveHandle.ballProjectionEstimate) < SPQR::GOALIE_FAR_LIMIT_Y)
            {
                if( theDiveHandle.diveTime != -1 && technicalDiveTime < SPQR::GOALIE_DIVE_TIME_TOLERANCE)
                {
                    SPQR_INFO("[GOALIE learner] dive time: " << technicalDiveTime);
					if(theDiveHandle.ballProjectionEstimate > SPQR::GOALIE_CLOSE_LIMIT_Y)
					{
                        //Technical goalie computation BEGIN
                        libCodeRelease.diveBool = true;
						//Technical goalie computation END
						goto goalieDiveLeft;
					}
					else if(theDiveHandle.ballProjectionEstimate < -SPQR::GOALIE_CLOSE_LIMIT_Y)
					{
						//Technical goalie computation BEGIN    
                        libCodeRelease.diveBool = true;
						//Technical goalie computation END
                        goto goalieDiveRight;
					}
					else
						goto stopBall;
                }
            }
        }
        action
        {
            if(libCodeRelease.timeSinceBallWasSeen() < 2000)
                theHeadControlMode = HeadControl::lookAtBall;
            else
                theHeadControlMode = HeadControl::lookAtGlobalBall;

            Stand();
        }
    }

    state(goalieDiveLeft)
    {
        transition
        {
            if (state_time > SPQR::GOALIE_DIVE_TIME)
            {
                goto mainLoop;
            }
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            GoalieSaveLeft();
        }
    }

    state(goalieDiveRight)
    {
        transition
        {
            if (state_time > SPQR::GOALIE_DIVE_TIME)
            {
                goto mainLoop;
            }
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            GoalieSaveRight();
        }
    }

	state(stopBall)
	{
		transition
        {
			if (state_time > SPQR::GOALIE_DIVE_TIME)
                goto mainLoop;
		}
		action
		{
			theHeadControlMode = HeadControl::lookForward;
			StopBall();
		}
	}

}
