#pragma once

#include <cmath>

namespace PTracking
{
	/**
	 * @struct Point2
	 * 
	 * @brief Struct that represents a cartesian point without orientation.
	 */
	template <typename Numeric> struct Point2
	{
		/**
		 * @brief value of the ordinate.
		 */
		Numeric x;
		
		/**
		 * @brief value of the abscissa.
		 */
		Numeric y;
		
		/**
		 * @brief Empty constructor.
		 * 
		 * It initializes x and y with the default value 0.
		 */
		Point2() : x(0), y(0) {;}
		
		/**
		 * @brief Constructor that takes x and y as initialization values.
		 * 
		 * It initializes x and y with the values given in input.
		 * 
		 * @param x value of the ordinate.
		 * @param y value of the abscissa.
		 */
		Point2(Numeric x, Numeric y) : x(x), y(y) {;}
		
		/**
		 * @brief Constructor that takes a Point as initialization value.
		 * 
		 * It initializes x and y with the point given in input.
		 * 
		 * @param p reference to the new point.
		 */
		Point2(const Point2<Numeric>& p) : x(p.x), y(p.y) {;}
		
		/**
		 * @brief Operator that computes the difference of two points.
		 * 
		 * @param p reference to the point that we want to subtract to the current one.
		 * 
		 * @return a new point obtained by subtracting, for each component, the current point and p.
		 */
		Point2 operator- (const Point2& p) const { return Point2(x - p.x,y - p.y); }
		
		/**
		 * @brief Operator that computes the sum of two points.
		 * 
		 * @param p reference to the point that we want to sum to the current one.
		 * 
		 * @return a new point obtained by summing, for each component, the current point and p.
		 */
		Point2 operator+ (const Point2& p) const { return Point2(x + p.x,y + p.y); }
		
		/**
		 * @brief Operator that computes the multiplication of the current point by a factor given in input.
		 * 
		 * @param n reference to the multiplication factor.
		 * 
		 * @return a new point obtained by multiplying each component by n.
		 */
		Point2 operator* (const Numeric& n) const { return Point2(x * n,y * n); }
		
		/**
		 * @brief Operator that computes the division of the current point by a factor given in input.
		 * 
		 * @param n reference to the division factor.
		 * 
		 * @return a new point obtained by dividing each component by n.
		 */
		Point2 operator/ (const Numeric& n) const { return Point2(x / n,y / n); }
		
		/**
		 * @brief Operator that computes the difference of two points, updating the current one with the result.
		 * 
		 * @param p reference to the point that we want to subtract to the current one.
		 * 
		 * @return a reference to the current point obtained by subtracting, for each component, the current point and p.
		 */
		Point2& operator-= (const Point2& p) { x -= p.x; y -= p.y; return *this; }
		
		/**
		 * @brief Operator that computes the sum of two points, updating the current one with the result.
		 * 
		 * @param p reference to the point that we want to sum to the current one.
		 * 
		 * @return a reference to the current point obtained by summing, for each component, the current point and p.
		 */
		Point2& operator+= (const Point2& p) { x += p.x; y += p.y; return *this; }
		
		/**
		 * @brief Operator that checks if two points are equal.
		 * 
		 * @param p reference to the point that we want to compare with the current one.
		 * 
		 * @return \b true if the current point and p are equal, \b false otherwise.
		 */
		bool operator== (const Point2& p) const { return (x == p.x) && (y == p.y); }
		
		/**
		 * @brief Operator that checks if two points are not equal.
		 * 
		 * @param p reference to the point that we want to compare with the current one.
		 * 
		 * @return \b true if the current point and p are not equal, \b false otherwise.
		 */
		bool operator!= (const Point2& p) const { return !(p == *this); }
		
		/**
		 * @brief Operator that checks if the current point is less than the one given in input.
		 * 
		 * The comparison of the two points begins by checking if the x component of the two points is equal. If not, it will checks the y component.
		 * 
		 * @param p reference to the point that we want to check.
		 * 
		 * @return \b true if the current point is less than p, \b false otherwise.
		 */
		bool operator< (const Point2& p) const { return (x < p.x) || ((x == p.x) && (y < p.y)); }
		
		/**
		 * @brief Function that computes the Euclidean distance of the current point from the origin.
		 * 
		 * @return the Euclidean distance of the current point from the origin.
		 */
		float mod() const { return sqrt(mod2()); }
		
		/**
		 * @brief Function that computes the squared Euclidean distance of the current point from the origin.
		 * 
		 * @return the squared Euclidean distance of the current point from the origin.
		 */
		float mod2() const { return (x * x) + (y * y); }
		
		/**
		 * @brief Function that computes the angle between the y and x coordinates.
		 * 
		 * @return the angle between the y and x coordinates.
		 */
		float theta() const { return ((x != 0) || (y != 0)) ? atan2(y,x) : 0; }
	};
	
	typedef Point2<float> Point2f;
}
