#pragma once

#include "../Filters/ObjectSensorReading.h"
#include "../SensorMaps/BasicSensorMap.h"
#include <Utils/Point2of.h>
#include <Utils/PolarPoint.h>

namespace PTracking
{
	/**
	 * @class BasicSensorModel
	 * 
	 * @brief Class that implements a sensor model.
	 */
	class BasicSensorModel : public manfield::SensorModel
	{
		private:
			/**
			 * @brief value representing the default linear velocity (in m/s) of a target.
			 */
			float defaultLinearVelocity;
			
			/**
			 * @brief value representing the linear sigma of the distribution.
			 */
			float sigmaRho;
			
			/**
			 * @brief value representing the angular sigma of the distribution.
			 */
			float sigmaTheta;
			
			/**
			 * @brief value representing the current linear velocity (in m/s) of the target.
			 */
			float linearVelocity;
			
		public:
			/**
			 * @brief Constructor that takes the sensor model type as initialization value.
			 * 
			 * It initializes the type of the sensor model with the one given in input.
			 * 
			 * @param type sensor model type.
			 */
			BasicSensorModel(const std::string& type = "BasicSensorModel");
			
			/**
			 * @brief Destructor.
			 */
			virtual ~BasicSensorModel();
			
			/**
			 * @brief Function that computes the current linear velocity of the target.
			 * 
			 * @param initialTimestamp time when the (i-1)-th target estimation has been performed.
			 * @param currentTimestamp time when the i-th target estimation has been performed.
			 */
			void calculateLinearVelocity(unsigned long initialTimestamp, unsigned long currentTimestamp);
			
			/**
			 * @brief Function that reads a config file in order to initialize several configuration parameters.
			 * 
			 * @param filename file to be read.
			 * @param sensor pointer to the sensor to be used.
			 */
			virtual void configure(const std::string& filename, GMapping::Sensor* sensor = 0);
			
			/**
			 * @brief Function that updates the particles' weight by calculating the new likelihood distribution given a set of observations.
			 * 
			 * @param reading pointer to the current observations.
			 * @param particlesBegin reference to the first particle of the distribution.
			 * @param particlesEnd reference to the last particle of the distribution.
			 */
			virtual void likelihood(GMapping::SensorReading* reading, PointIterator& particlesBegin, PointIterator& particlesEnd) const;
			
			/**
			 * @brief Function that computes the weight of a particle given a set of observations.
			 * 
			 * @param map pointer to the model of the map.
			 * @param pose reference to the cartesian position of the particle.
			 * @param observations reference to the current observations.
			 * 
			 * @return the new weight of the particle.
			 */
			virtual float likelihood(BasicSensorMap* map, PointWithVelocity& pose, std::vector<ObjectSensorReading::Observation>& observations) const;
			
			/**
			 * @brief Macro that defines the default clone function.
			 */
			MODEL_DEFAULT_CLONE(BasicSensorModel)
	};
}
