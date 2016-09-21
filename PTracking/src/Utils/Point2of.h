#pragma once

#include "Point2f.h"

namespace PTracking
{
	/**
	 * @class Point2o
	 * 
	 * @brief Class that represents a cartesian point with orientation.
	 */
	template <typename Numeric> class Point2o : public Point2<Numeric>
	{
		public:
			/**
			 * @brief orientation of the point.
			 */
			Numeric theta;
			
			/**
			 * @brief Empty constructor.
			 * 
			 * It initializes x, y and theta with the default value 0.
			 */
			Point2o() : Point2<Numeric>(), theta(0) {;}
			
			/**
			 * @brief Constructor that takes x, y and theta as initialization values.
			 * 
			 * It initializes x, y and theta with the values given in input.
			 * 
			 * @param x value of the ordinate.
			 * @param y value of the abscissa.
			 * @param theta orientation of the point.
			 */
			Point2o(Numeric x, Numeric y, float theta) : Point2<Numeric>(x,y), theta(theta) {;}
			
			/**
			 * @brief Constructor that takes a point and the orientation as initialization value.
			 * 
			 * It initializes x, y and theta with the values given in input.
			 * 
			 * @param p reference to the new point.
			 * @param theta orientation of the point.
			 */
			explicit Point2o(const Point2<Numeric>& p, float theta = 0) : Point2<Numeric>(p), theta(theta) {;}
			
			/**
			 * @brief Constructor that takes a Point as initialization value.
			 * 
			 * It initializes x, y and theta with the point given in input.
			 * 
			 * @param p reference to the new point.
			 */
			Point2o(const Point2o<Numeric>& p) : Point2<Numeric>(p.x,p.y), theta(p.theta) {;}
			
			/**
			 * @brief Operator that computes the difference of two points with orientation.
			 * 
			 * @param p reference to the point that we want to subtract to the current one.
			 * 
			 * @return a new point obtained by subtracting, for each component, the current point and p.
			 */
			Point2o<Numeric> operator- (const Point2o<Numeric>& p) const { return Point2o<Numeric>(this->x - p.x,this->y - p.y,this->theta - p.theta); }
			
			/**
			 * @brief Operator that computes the sum of two points with orientation.
			 * 
			 * @param p reference to the point that we want to sum to the current one.
			 * 
			 * @return a new point obtained by summing, for each component, the current point and p.
			 */
			Point2o<Numeric> operator+ (const Point2o<Numeric>& p) const { return Point2o<Numeric>(this->x + p.x,this->y + p.y,theta + p.theta); }
			
			/**
			 * @brief Operator that computes the multiplication of the current point by a factor given in input.
			 * 
			 * @param n reference to the multiplication factor.
			 * 
			 * @return a new point obtained by multiplying each component by n.
			 */
			Point2o<Numeric> operator* (const Numeric& n) const { return Point2o<Numeric>(this->x * n,this->y * n,this->theta * n); }
	};
	
	typedef Point2o<float> Point2of;
}
