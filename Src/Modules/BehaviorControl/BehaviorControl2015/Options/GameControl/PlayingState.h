
//#define NO_WIFI_CHALLENGE
//#define SINGLE_ROLE

option(PlayingState)
{
    initial_state(play)
    {
        action
        {
#ifdef NO_WIFI_CHALLENGE
			if (theRobotInfo.number == 1) 
				Receiver();
			else
				Sender();
#else 
  #ifdef SINGLE_ROLE
//            PassBall();

            //Stand();
            PenaltyStriker();
  #else
  			//                if(theOwnTeamInfo.teamColor == TEAM_BLUE && theRobotInfo.number == 2)
  			//                    DefenderLearner();
  			//                else
  			{
  				if(theContextCoordination.robotRole == ContextCoordination::goalie)
  					Goalie();
  				else if(theContextCoordination.robotRole == ContextCoordination::defender /*&& theOwnTeamInfo.teamColor == TEAM_RED*/)
  					DefenderPF();
  				else if(theContextCoordination.robotRole == ContextCoordination::jolly)
  					JollyPF();
  				else if(theContextCoordination.robotRole == ContextCoordination::supporter)
  					SupporterPF();
  				else if(theContextCoordination.robotRole == ContextCoordination::striker)
  					Striker();
  
  				//Search
  				else if(theContextCoordination.robotRole == ContextCoordination::searcher_1)
  					Searcher(1);
  				else if(theContextCoordination.robotRole == ContextCoordination::searcher_2)
  					Searcher(2);
  				else if(theContextCoordination.robotRole == ContextCoordination::searcher_3)
  					Searcher(3);
  
  				/// throw-In
  				else if(theContextCoordination.robotRole == ContextCoordination::throwin_searcher_1)
  					Throwin(1);
  				else if(theContextCoordination.robotRole == ContextCoordination::throwin_searcher_2)
  					Throwin(2);
  				else if(theContextCoordination.robotRole == ContextCoordination::throwin_searcher_3)
  					Throwin(3);
  				else if(theContextCoordination.robotRole == ContextCoordination::guard)
  					Guard();
  			}
  #endif
#endif

        }
    }
}

