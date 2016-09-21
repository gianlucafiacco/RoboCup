#pragma once

#include "Point2of.h"

namespace PTracking
{
	/**
	 * @class PointWithVelocity
	 * 
	 * @brief Class that represents a cartesian point with orientation and velocity.
	 */
	class PointWithVelocity
	{
		public:
			/**
			 * @brief cartesian point with orientation.
			 */
			Point2of pose;
			
			/**
			 * @brief velocity in the x axis.
			 */
			float vx;
			
			/**
			 * @brief velocity in the y axis.
			 */
			float vy;
			
			/**
			 * @brief Empty constructor.
			 * 
			 * It initializes the cartesian point and the velocities with the default value 0.
			 */
			PointWithVelocity() : pose(), vx(0.0), vy(0.0) {;}
	};
}
