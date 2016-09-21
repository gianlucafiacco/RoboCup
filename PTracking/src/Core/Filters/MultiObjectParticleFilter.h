#pragma once

#include "ObjectSensorReading.h"
#include <Utils/Timestamp.h>
#include <Manfield/filters/particlefilter.h>
#include <set>

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
	 * @class MultiObjectParticleFilter
	 * 
	 * @brief Class that implements the particle filter designed by Wu et al.
	 * 
	 * Reference paper: "Boosted Interactively Distributed Particle Filter for Automatic Multi-Object Tracking". In 15th IEEE International Conference on Image Processing, 2008.
	 */
	class MultiObjectParticleFilter : public manfield::ParticleFilter
	{
		private:
			/**
			 * @brief maximum linear velocity admissible for a target.
			 */
			static const float MAX_FEASIBLE_LINEAR_VELOCITY = 1.0;
			
			/**
			 * @brief matrix representing the state of the particle filters.
			 */
			std::vector<std::vector<std::pair<int,unsigned long> > > stateMergeMatrix;
			
			/**
			 * @brief bank of particle filters.
			 */
			std::vector<std::pair<ParticleFilter*,unsigned long> > particleFilterVector;
			
			/**
			 * @brief pointer to the clustering algorithm.
			 */
			Clusterizer* clusterizer;
			
			/**
			 * @brief type of the clustering algortithm.
			 */
			std::string clusteringAlgorithm;
			
			/**
			 * @brief name of the parameters config file.
			 */
			std::string parametersFile;
			
			/**
			 * @brief threshold used for the data association phase.
			 */
			float qualityThreshold;
			
			/**
			 * @brief timestamp of the current iteration.
			 */
			unsigned long currentTimestamp;
			
			/**
			 * @brief timestamp of the last time when the history of the estimations has been cleaned.
			 */
			unsigned long lastCheck;
			
			/**
			 * @brief maximum time to wait before merging multiple overlapped particle filters. 
			 */
			unsigned int maxTimePassedFromLastOverlap;
			
			/**
			 * @brief minimim number of particles to consider a cluster as significant.
			 */
			unsigned int minElementSignificantCluster;
			
			/**
			 * @brief initial value for the number of targets to be tracked.
			 */
			unsigned int targetNumber;
			
			/**
			 * @brief threshold value to consider multiple particle filters overlapped.
			 */
			int thresholdOverlap;
			
			/**
			 * @brief Function that deletes old particle filters.
			 * 
			 * @param validityTime maximum time before considering a particle filter invalid.
			 */
			void checkParticleFilterVectorForCleaning(unsigned long validityTime);
			
			/**
			 * @brief Function that checks if a set of particle filters have to be deleted or not.
			 * 
			 * @param filterToDelete reference to the set of particle filters to be checked.
			 */
			void checkStateMergeMatrixForCleaning(std::set<int>& filterToDelete);
			
			/**
			 * @brief Function that finds the observations close to the particle filter given in input.
			 * 
			 * @param f pointer to the particle filter to be checked.
			 * @param readings reference to the observations gathered from the sensors between the previous and current iteration.
			 * @param readingAssociated reference to the observations associated.
			 * 
			 * @return the observations close to the particle filter.
			 */
			std::vector<ObjectSensorReading::Observation> findNearestObservations(ParticleFilter* f, ObjectSensorReading& readings, set<int>& readingAssociated) const;
			
			/**
			 * @brief Function that checks if an existing particle filter is close to the observation given in input.
			 * 
			 * @param observation reference to the observation to be checked.
			 * 
			 * @return \b true if an existing particle filter is found, \b false otherwise.
			 */
			bool isThereAlreadyNearFilter(const PolarPoint& observation) const;
			
			/**
			 * @brief Function that merges all the overlapped particle filters.
			 */
			void merge();
			
			/**
			 * @brief Function that splits a particle filter of the bank.
			 * 
			 * @param c reference to the vector of estimations contained in the particle filter.
			 * @param i index of the particle filter in the bank.
			 */
			void split(std::vector<std::pair<PoseParticleVector,Point2of> >& c, unsigned int i);
			
		public:
			/**
			 * @brief Constructor that takes the type of the particle filter as initialization value.
			 * 
			 * It initializes the type of the particle filter with the one given in input.
			 * 
			 * @param type particle filter type.
			 */
			MultiObjectParticleFilter(const std::string& type = "MultiObjectParticleFilter");
			
			/**
			 * @brief Destructor.
			 * 
			 * It deallocates the memory of the clusterizer and of the bank of filters.
			 */
			~MultiObjectParticleFilter();
			
			/**
			 * @brief Function that reads a config file in order to initialize several configuration parameters.
			 * 
			 * @param filename file to be read.
			 */
			void configure(const std::string& filename);
			
			/**
			 * @brief Function that returns the estimations without identity performed by the agents.
			 * 
			 * @return a reference to the estimations without identity performed by the agents.
			 */
			std::vector<std::pair<PoseParticleVector,Point2of> >& getClusters();
			
			/**
			 * @brief Function that returns the sensor used by the agent.
			 * 
			 * @return a reference to the sensor used by the agent.
			 */
			const BasicSensor& getSensor() const { return *static_cast<const BasicSensor*>(ParticleFilter::getSensor()); }
			
			/**
			 * @brief Function that returns the significant estimations without identity performed by the agents.
			 * 
			 * @return the significant estimations without identity performed by the agents.
			 */
			std::vector<std::pair<PoseParticleVector,Point2of> > getSignificantClusters() const;
			
			/**
			 * @brief Function that initializes the bank of filters.
			 */
			void init();
			
			/**
			 * @brief Function that creates the new likelihood distribution using the observations gathered from the agent's sensors.
			 * 
			 * @param readings reference to the observations gathered from the sensors between the previous and current iteration.
			 */
			void observe(ObjectSensorReading& readings);
			
			/**
			 * @brief Function that updates the motion model of the particle filter.
			 * 
			 * @param targetSeen true if a target has been seen, false otherwise.
			 * @param initialTimestamp reference to the timestamp of the previous iteration.
			 * @param current reference to the timestamp of the current iteration.
			 */
			void predict(bool targetSeen, const Timestamp& initialTimestamp, const Timestamp& current);
			
			/**
			 * @brief Macro that defines the default clone function.
			 */
			FILTER_DEFAULT_CLONE(MultiObjectParticleFilter)
	};
}
