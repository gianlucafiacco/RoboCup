#pragma once

#include "ObjectSensorReading.h"
#include "../Sensors/BasicSensor.h"
#include <Utils/Point2f.h>
#include <ThirdParty/GMapping/sensor/sensor_base/sensorreading.h>
#include <Manfield/filters/gmlocalizer/structs.h>

namespace PTracking
{
	/**
	 * @class ObjectSensorReadingMultiAgent
	 * 
	 * @brief Class that defines the observations exchanged by the team of agents.
	 */
	class ObjectSensorReadingMultiAgent : public GMapping::SensorReading
	{
		public:
			/**
			 * @struct ObservationMultiAgent
			 * 
			 * @brief Struct representing an estimation exchanged by the team of agents. 
			 */
			struct ObservationMultiAgent
			{
				/**
				 * @brief vector of estimations with identity performed by an agent.
				 */
				std::map<int,std::pair<ObjectSensorReading::Observation,Point2f> > estimationsWithModels;
				
				/**
				 * @brief timestamp of the estimations.
				 */
				unsigned long timestamp;
			};
			
			/**
			 * @brief Empty constructor.
			 */
			ObjectSensorReadingMultiAgent();
			
			/**
			 * @brief Destructor.
			 */
			~ObjectSensorReadingMultiAgent();
			
			/**
			 * @brief Function that returns the timestamp of the estimations.
			 * 
			 * @return the timestamp of the estimations.
			 */
			inline unsigned long getEstimationsTimestamp() const { return observationMultiAgent.timestamp; }
			
			/**
			 * @brief Function that returns the estimations performed by the team of agents.
			 * 
			 * @return a reference to the vector of estimations performed by the team of agents.
			 */
			inline const std::map<int,std::pair<ObjectSensorReading::Observation,Point2f> >& getEstimationsWithModels() const { return observationMultiAgent.estimationsWithModels; }
			
			/**
			 * @brief Function that returns the sensor used by the team of agents.
			 * 
			 * @return a reference to the sensor used by the team of agents.
			 */
			inline const BasicSensor& getSensor() const { return sensor; }
			
			/**
			 * @brief Function that updates the timestamp of the estimations.
			 * 
			 * @param timestamp new timestamp of the estimations.
			 */
			void setEstimationsTimestamp(unsigned long timestamp);
			
			/**
			 * @brief Function that updates the estimations performed by the team of agents.
			 * 
			 * @param estimationsWithModels reference to a new vector of estimations.
			 */
			void setEstimationsWithModels(const std::map<int,std::pair<ObjectSensorReading::Observation,Point2f> >& estimationsWithModels);
			
			/**
			 * @brief Function that sets a new sensor.
			 * 
			 * @param s reference to a new sensor.
			 */
			void setSensor(const BasicSensor& s);
			
		private:
			/**
			 * @brief estimations performed by an agent.
			 */
			ObservationMultiAgent observationMultiAgent;
			
			/**
			 * @brief sensor used by the team of agents.
			 */
			BasicSensor sensor;
	};
}
