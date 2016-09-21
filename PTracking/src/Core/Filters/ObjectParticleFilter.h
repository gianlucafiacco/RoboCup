#pragma once

#include "ObjectSensorReading.h"
#include <Utils/Timestamp.h>
#include <Manfield/filters/particlefilter.h>

namespace PTracking
{
	/**
	 * @brief Forward declaration.
	 */
	class Clusterizer;
}

namespace PTracking
{
	/**
	 * @class ObjectParticleFilter
	 * 
	 * @brief Class that implements the particle filter used in the local estimation layer.
	 */
	class ObjectParticleFilter : public manfield::ParticleFilter
	{
		private:
			/**
			 * @brief maximum linear velocity admissible for a target.
			 */
			static const float FEASIBLE_LINEAR_VELOCITY	= 5.0;
			
			/**
			 * @brief time to wait before promoting an observation coming from the agent's sensors.
			 */
			static const int TIME_TO_WAIT_BEFORE_PROMOTING = 1000;
			
			/**
			 * @brief time to wait before deleting an estimation no longer associated to an observation coming from the agent's sensors.
			 */
			static const unsigned int TIME_TO_WAIT_BEFORE_DELETING = 300;
			
			/**
			 * @brief map of estimations having both an identity and a model performed by the agent.
			 */
			std::map<int,std::pair<ObjectSensorReading::Observation,Point2f> > estimatedTargetModelsWithIdentity;
			
			/**
			 * @brief map representing the timestamp on which an estimation has been updated.
			 */
			std::map<int,Timestamp> estimationsUpdateTime;
			
			/**
			 * @brief vector representing all the models of the estimations performed by the agent.
			 */
			std::vector<std::pair<ObjectSensorReading::Model,int> > targetModels;
			
			/**
			 * @brief vector of pending observations.
			 */
			std::vector<std::pair<Point2of,std::pair<Timestamp,Timestamp> > > pendingObservations;
			
			/**
			 * @brief vector representing the mapping between observations and estimations (used for debugging).
			 */
			std::vector<std::pair<Point2of,Point2of> > observationsMapping;
			
			/**
			 * @brief pointer to the clustering algorithm.
			 */
			Clusterizer* clusterizer;
			
			/**
			 * @brief timestamp of the last time when the target's number should be decreased.
			 */
			Timestamp lastTimeShouldBeDecreased;
			
			/**
			 * @brief type of the clustering algortithm.
			 */
			std::string clusteringAlgorithm;
			
			/**
			 * @brief threshold used for the data association phase.
			 */
			float closenessThreshold;
			
			/**
			 * @brief timestamp of the current iteration.
			 */
			unsigned long currentTimestamp;
			
			/**
			 * @brief timestamp of the last time when the history of the estimations has been cleaned.
			 */
			unsigned long lastCheck;
			
			/**
			 * @brief number of the observations provided by the agent's sensors that have been either associated to an existing cluster or definitely promoted.
			 */
			unsigned int numberOfObservationAssociatedAndPromoted;
			
			/**
			 * @brief maximum identity value assigned to an estimation.
			 */
			int maxIdentityNumber;
			
			/**
			 * @brief initial value for the number of targets to be tracked.
			 */
			int targetNumber;
			
			/**
			 * @brief true means that the observe step is needed, otherwise it is not needed yet.
			 */
			bool observeNeeded;
			
			/**
			 * @brief Function that checks if two models are similar.
			 * 
			 * @param observationModel reference to the model of the observation.
			 * @param storedModel reference to a model already present in the bank of models.
			 * 
			 * @return \b true if the models are similar, \b false otherwise.
			 */
			bool areModelsSimilar(const ObjectSensorReading::Model& observationModel, const ObjectSensorReading::Model& storedModel);
			
			/**
			 * @brief Function that adds particles for representing a new observation once promoted.
			 * 
			 * @param readings reference to the observations gathered from the sensors between the previous and current iteration.
			 */
			void addParticlesInInterestingPoints(ObjectSensorReading& readings);
			
			/**
			 * @brief Function that updates the estimations having both an identity and a model performed by the agent.
			 * 
			 * @param readings reference to the observations gathered from the sensors between the previous and current iteration.
			 */
			void updateTargetIdentity(ObjectSensorReading& readings);
			
		public:
			/**
			 * @brief Constructor that takes the type of the particle filter as initialization value.
			 * 
			 * It initializes the type of the particle filter with the one given in input.
			 * 
			 * @param type particle filter type.
			 */
			ObjectParticleFilter(const std::string& type = "ObjectParticleFilter");
			
			/**
			 * @brief Destructor.
			 * 
			 * It deallocates the memory of the clusterizer.
			 */
			~ObjectParticleFilter();
			
			/**
			 * @brief Function that checks whether a re-initialization of the particle filter is needed.
			 */
			void checkFilterReinitializationNeeded();
			
			/**
			 * @brief Function that reads a config file in order to initialize several configuration parameters.
			 * 
			 * @param filename file to be read.
			 */
			void configure(const std::string& filename);
			
			/**
			 * @brief Function that returns the pointer to the clustering algorithm.
			 * 
			 * @return the pointer to the clustering algorithm.
			 */
			inline Clusterizer* getClusterizer() const { return clusterizer; }
			
			/**
			 * @brief Function that returns the mapping between observations and estimations (used for debugging).
			 * 
			 * @return the mapping between observations and estimations.
			 */
			inline const std::vector<std::pair<Point2of,Point2of> >& getObservationsMapping() const { return observationsMapping; }
			
			/**
			 * @brief Function that returns the estimations having both an identity and a model performed by the agent.
			 * 
			 * @return the vector of estimations having both an identity and a model performed by the agent.
			 */
			inline std::map<int,std::pair<ObjectSensorReading::Observation,Point2f> > getEstimationsWithModel() const { return estimatedTargetModelsWithIdentity; }
			
			/**
			 * @brief Function that returns the sensor used by the agent.
			 * 
			 * @return a reference to the sensor used by the agent.
			 */
			inline const BasicSensor& getSensor() const { return *static_cast<const BasicSensor*>(ParticleFilter::getSensor()); }
			
			/**
			 * @brief Function that normalizes the weight of the particles.
			 */
			void normalizeWeight();
			
			/**
			 * @brief Function that creates the new likelihood distribution using the observations gathered from the agent's sensors.
			 * 
			 * @param readings reference to the observations gathered from the sensors between the previous and current iteration.
			 */
			void observe(ObjectSensorReading& readings);
			
			/**
			 * @brief Function that updates the motion model of the particle filter.
			 * 
			 * @param newRobotPose reference to the position of the robot when the observations have been acquired.
			 * @param oldRobotPose reference to the position of the robot in the previous iteration.
			 * @param targetSeen true if a target has been seen, false otherwise.
			 * @param initialTimestamp reference to the timestamp of the previous iteration.
			 * @param current reference to the timestamp of the current iteration.
			 */
			void predict(const Point2of& newRobotPose, const Point2of& oldRobotPose, bool targetSeen, const Timestamp& initialTimestamp, const Timestamp& current);
			
			/**
			 * @brief Function that resamples the particles.
			 */
			void resample();
			
			/**
			 * @brief Function that resets the weight of the particles by setting their value to 1.
			 */
			void resetWeight();
			
			/**
			 * @brief Macro that defines the default clone function.
			 */
			FILTER_DEFAULT_CLONE(ObjectParticleFilter)
	};
}
