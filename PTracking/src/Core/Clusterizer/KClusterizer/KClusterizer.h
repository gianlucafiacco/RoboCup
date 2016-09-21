#pragma once

#include "../Clusterizer.h"
#include <Utils/PriorityBuffer.h>

namespace PTracking
{
	/**
	 * @class KClusterizer
	 * 
	 * @brief Class that implements a clustering algorithm called KClusterizer.
	 * 
	 * KClusterizer does not require any initialization, it is very fast and all the obtained clusters have a specified maximum radius given in input. Indeed, the
	 * desired number of clusters \a k is initially set to 1 and dynamically adapted by the architecture, in which the KClusterizer algorithm is used, in order to
	 * reflect the number of objects to be tracked. 
	 */
	class KClusterizer : public Clusterizer
	{
		private:
			/**
			 * @struct Cluster
			 * 
			 * @brief Struct that represents a cluster.
			 */
			struct Cluster
			{
				/**
				 * @brief centroid of the cluster.
				 */
				PoseParticle k;
				
				/**
				 * @brief particles contained in the cluster.
				 */
				std::vector<int> indices;
			};
			
			/**
			 * @brief spaced out particles that represent the centroids of the clusters.
			 */
			PriorityBuffer<PoseParticle> kpoints;
			
			/**
			 * @brief value of the current number of clusters \a k used during the clustering phase.
			 */
			int maxClusters;
			
			/**
			 * @brief Function that checks if two particles are far each other within a maximum threshold distance.
			 * 
			 * @param p1 reference to the first particle to be checked.
			 * @param p2 reference to the second particle to be checked.
			 * @param qualityThreshold maximum threshold distance.
			 * 
			 * @return \b true if the particles are far each other, \b false otherwise.
			 */
			bool isFarFrom(const PoseParticle& p1, const PoseParticle& p2, float qualityThreshold) const;
			
			/**
			 * @brief Function that checks if the particle is far from all the others within a maximum threshold distance.
			 * 
			 * @param p reference to the particle to be checked.
			 * @param qualityThreshold maximum threshold distance.
			 * 
			 * @return \b true if the particle is far from all the others, \b false otherwise.
			 */
			bool isFarFromAll(const PoseParticle& p, float qualityThreshold) const;
			
		public:
			/**
			 * @brief Constructor that takes the desired maximum number of clusters as initialization value.
			 * 
			 * It initializes the desired number of clusters with the one given in input.
			 * 
			 * @param maxClusters desired maximum number of clusters.
			 */
			KClusterizer(int maxClusters);
			
			/**
			 * @brief Destructor.
			 */
			~KClusterizer();
			
			/**
			 * @brief Function that clusterizes a vector of particles creating a set of clusters with a maximum radius given in input.
			 * 
			 * @param particleVector reference to the particles' vector that have to be clusterized.
			 * @param qualityThreshold maximum radius of a cluster.
			 */
			void clusterize(PoseParticleVector& particleVector, float qualityThreshold);
			
			/**
			 * @brief Function that returns the number of clusters \a k used by the clustering algorithm to clusterize the particles.
			 * 
			 * @return the desired number of clusters \a k.
			 */
			int getCurrentClusterNumber() const { return maxClusters; }
			
			/**
			 * @brief Function that updates the value of \a k representing the desired number of clusters.
			 * 
			 * @param k new value of the desired clusters.
			 */
			void setMaxClusterNumber(int k);
	};
}
