#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(RobotPoseSpqrFiltered,
{
	public:
	
		float x,        	/**< The X position of the filtered robot pose */
        (float) y,	    	/**< The Y position of the filtered robot pose */
        (float) theta,    	/**< The THETA position of the filtered robot pose */

        RobotPoseSpqrFiltered() = default;
});

STREAMABLE(RobotPoseSpqrFilteredCompressed,
{
public:
  RobotPoseSpqrFilteredCompressed() = default;
  RobotPoseSpqrFilteredCompressed(const RobotPoseSpqrFiltered& robotPoseSpqrFiltered);
  operator RobotPoseSpqrFiltered() const,
	(float) x,
	(float) y,
	(float) theta,
});
