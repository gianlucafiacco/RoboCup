#include "PTracker.h"

int main (int argc, char** argv)
{
	if (argc != 2)
	{
		ERR("Usage: ./PTracker <observation-file>." << endl);
		
		exit(-1);
	}
	
	PTracker pTracker;
	
	pTracker.exec(argv[1]);
	
	return 0;
}
