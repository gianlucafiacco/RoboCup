#pragma once

#include <Utils/Point2f.h>
#include <Manfield/sensormodel.h>

namespace PTracking
{
	/**
	 * @class BasicSensorMap
	 * 
	 * @brief Class that implements a model of a map.
	 */
	class BasicSensorMap : public manfield::SensorMap
	{
		public:
			/**
			 * @brief Empty Constructor.
			 */
			BasicSensorMap();
			
			/**
			 * @brief Destructor.
			 */
			~BasicSensorMap();
			
			/**
			 * @brief Function that returns a random point inside the world.
			 * 
			 * @return the random point generated.
			 */
			PTracking::Point2f getRandomPointInWorld() const;
			
			/**
			 * @brief Function that checks if a given point is inside the world.
			 * 
			 * @param x ordinate value of the point to be checked.
			 * @param y abscissa value of the point to be checked.
			 * 
			 * @return \b true if the point is inside the world, \b false otherwise.
			 */
			bool isInsideWorld(float x, float y) const;
			
			/**
			 * @brief Function that inserts a new sensor into the sensor's bank.
			 * 
			 * @param sensorPose reference to the position of the sensor to be added.
			 */
			inline void insertSensor(const PTracking::Point2f& sensorPose) { m_landmarks.push_back(sensorPose); }
			
			/**
			 * @brief Macro that defines the worldMin member variable and its get and set function.
			 */
			PARAM_SET_GET(PTracking::Point2f, worldMin, protected, public, public)
			
			/**
			 * @brief Macro that defines the worldMax member variable and its get and set function.
			 */
			PARAM_SET_GET(PTracking::Point2f, worldMax, protected, public, public)
			
			/**
			 * @brief Macro that defines the landmarks member variable and its get and set function.
			 */
			PARAM_SET_GET(std::vector<PTracking::Point2f>, landmarks, public, public, public)
	};
}
