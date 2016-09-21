#pragma once

#include "../Sensors/BasicSensor.h"
#include <Utils/Point2of.h>
#include <Utils/PolarPoint.h>
#include <ThirdParty/GMapping/sensor/sensor_base/sensorreading.h>
#include <vector>

namespace PTracking
{
	/**
	 * @class ObjectSensorReading
	 * 
	 * @brief Class that defines the observations coming from the agent's sensors.
	 */
	class ObjectSensorReading : public GMapping::SensorReading
	{
		public:
			/**
			 * @struct Model
			 * 
			 * @brief Struct representing either the model of the observation or the model of the estimation.
			 */
			struct Model
			{
				/**
				 * @brief width of the observation/estimation.
				 */
				int width;
				
				/**
				 * @brief height of the observation/estimation.
				 */
				int height;
				
				/**
				 * @brief barycenter of the observation/estimation.
				 */
				int barycenter;
				
				/**
				 * @brief RGB color of the observation/estimation.
				 */
				int color[3];
			};
			
			/**
			 * @struct Observation
			 * 
			 * @brief Struct representing either an observation provided by the agent's sensors or an estimation performed by the agent.
			 */
			struct Observation
			{
				/**
				 * @brief polar point representing an observation/estimation.
				 */
				PolarPoint observation;
				
				/**
				 * @brief standard deviation of the observation/estimation.
				 */
				Point2f sigma;
				
				/**
				 * @brief model of the estimation.
				 */
				Model model;
			};
			
			/**
			 * @brief Empty constructor.
			 */
			ObjectSensorReading();
			
			/**
			 * @brief Destructor.
			 */
			~ObjectSensorReading();
			
			/**
			 * @brief Function that returns either the vector representing the observation coming from the agent's sensors or the estimations performed by the agent.
			 * 
			 * @return a reference to either the vector of the observation coming from the agent's sensors or the estimations performed by the agent.
			 */
			std::vector<Observation>& getObservations() { return observations; }
			
			/**
			 * @brief Function that returns the position of the agent when the observations have been acquired.
			 * 
			 * @return a reference to the position of the agent when the observations have been acquired.
			 */
			const Point2of& getObservationsAgentPose() const { return observationsAgentPose; }
			
			/**
			 * @brief Function that returns the sensor used by the agent.
			 * 
			 * @return a reference to the sensor used by the agent.
			 */
			const BasicSensor& getSensor() const { return sensor; }
			
			/**
			 * @brief Function that updates the vector of observations coming from the agent's sensors.
			 * 
			 * @param obs reference to the vector of observations coming from the agent's sensors.
			 */
			void setObservations(const std::vector<Observation>& obs);
			
			/**
			 * @brief Function that updates the circular buffer of the observations coming from the agent's sensors by checking a maximum threshold distance.
			 * 
			 * @param targetPoints pointer to the observations provided by the agent's sensors.
			 * @param currentTargetIndex current index of the circular buffer.
			 * @param lastCurrentTargetIndex last current index of the circular buffer.
			 * @param lastNTargetPerception size of the circular buffer.
			 * @param maxRho maximum admissible distance of an observation coming from the agent's sensors.
			 */
			void setObservations(Observation targetPoints[], int currentTargetIndex, int lastCurrentTargetIndex, int lastNTargetPerception, float maxRho);
			
			/**
			 * @brief Function that updates the position of the agent when the observations have been acquired.
			 * 
			 * @param pose reference to the new position of the agent.
			 */
			void setObservationsAgentPose(const Point2of& pose);
			
			/**
			 * @brief Function that sets a new sensor.
			 * 
			 * @param s reference to a new sensor.
			 */
			void setSensor(const BasicSensor& s);
			
		private:
			/**
			 * @brief vector representing either the observation coming from the agent's sensors or the estimations performed by the agent.
			 */
			std::vector<Observation> observations;
			
			/**
			 * @brief the last position of the agent when the observations have been acquired.
			 */
			Point2of observationsAgentPose;
			
			/**
			 * @brief sensor used by the agent.
			 */
			BasicSensor sensor;
	};
}
