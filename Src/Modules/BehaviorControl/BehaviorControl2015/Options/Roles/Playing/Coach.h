

option(Coach)
{
	initial_state(start)
	{
		transition
		{
			STATE("start");
			
		}
		action
		{
            SpecialAction(SpecialActionRequest::coachSit);
		}
	}

}
