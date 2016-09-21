#pragma once

#include "Point2f.h"
#include <cmath>

namespace PTracking
{
	/**
	 * @struct PolarPoint
	 * 
	 * @brief Struct that represents a polar point.
	 */
	struct PolarPoint
	{
		/**
		 * @brief module of the point.
		 */
		float rho;
		
		/**
		 * @brief orientation of the point.
		 */
		float theta;
		
		/**
		 * @brief Constructor that takes rho and theta as initialization values.
		 * 
		 * It initializes rho and theta with the values given in input.
		 * 
		 * @param theta value of the orientation.
		 * @param rho value of the module.
		 */
		PolarPoint(float theta = 0.0, float rho = 0.0) : rho(rho), theta(theta) {;}
		
		/**
		 * @brief Operator that computes the sum of two polar points.
		 * 
		 * @param p reference to the polar point that we want to sum to the current one.
		 * 
		 * @return a new polar point obtained by summing the x and y component of the current polar point and p.
		 */
		PolarPoint operator+ (const PolarPoint& p) const
		{
			float x = rho * cos(theta) + p.rho * cos(p.theta);
			float y = rho * sin(theta) + p.rho * sin(p.theta);
			
			if ((x == 0) && (y == 0)) return PolarPoint(0,0);
			else return PolarPoint(atan2(y,x),sqrt((x * x) + (y * y)));
		}
		
		/**
		 * @brief Operator that computes the difference of two polar points.
		 * 
		 * @param p reference to the polar point that we want to subtract to the current one.
		 * 
		 * @return a new polar point obtained by subtracting the x and y component of the current polar point and p.
		 */
		PolarPoint operator- (const PolarPoint& p) const
		{
			PolarPoint np(p);
			
			np.theta -= M_PI;
			
			return *this + np;
		}
		
		/**
		 * @brief Operator that checks if the current polar point has an orientation less than the one of the polar point given in input.
		 * 
		 * @param p reference to the polar point that we want to compare with the current one.
		 * 
		 * @return \b true if the current polar point has an orientation less than the orientation of p, \b false otherwise.
		 */
		bool operator< (const PolarPoint& p) const { return theta < p.theta; }
		
		/**
		 * @brief Operator that checks if two polar points are equal.
		 * 
		 * @param p reference to the polar point that we want to compare with the current one.
		 * 
		 * @return \b true if the current polar point and p are equal, \b false otherwise.
		 */
		bool operator== (const PolarPoint& p) { return (theta == p.theta) && (rho == p.rho); }
		
		/**
		 * @brief Function that returns the cartesian point.
		 * 
		 * @return the cartesian form of the current polar point.
		 */
		Point2f getCartesian() const { return Point2f(cos(theta),sin(theta)) * rho; }
	};
}
