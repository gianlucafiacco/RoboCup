#pragma once

#include "../Clusterizer.h"

namespace PTracking
{
	/**
	 * @class QTClusterizer
	 * 
	 * @brief Class that implements the free-clustering algorithm called QT-Clustering.
	 */
	class QTClusterizer : public Clusterizer
	{
		private:
			/**
			 * @brief Function that finds the biggest cluster obtained after the clusterization phase.
			 * 
			 * @param allCandidateClusters reference to the vector of clusters obtained.
			 * @param indexParticlesToEliminate reference to a vector containing the indexes of particles to be deleted, once found the biggest cluster.
			 * 
			 * @return the particles representing the biggest cluster.
			 */
			PoseParticleVector getBiggestCluster(const std::vector<std::vector<std::pair<PoseParticle,int> > >& allCandidateClusters, std::vector<int>& indexParticlesToEliminate) const;
			
			/**
			 * @brief Function that checks if two particles are close each other within a maximum threshold distance.
			 * 
			 * @param p1 reference to the first particle to be checked.
			 * @param p2 reference to the second particle to be checked.
			 * @param qualityThreshold maximum threshold distance.
			 * 
			 * @return \b true if the particles are close each other, \b false otherwise.
			 */
			bool isNear(const PoseParticle& p1, const PoseParticle& p2, float qualityThreshold) const;
			
		public:
			/**
			 * @brief Empty constructor.
			 */
			QTClusterizer();
			
			/**
			 * @brief Destructor.
			 */
			~QTClusterizer();
			
			/**
			 * @brief Function that clusterizes a vector of particles creating a set of clusters with a maximum radius given in input.
			 * 
			 * @param particleVector reference to the particles' vector that have to be clusterized.
			 * @param qualityThreshold maximum radius of a cluster.
			 */
			void clusterize(PoseParticleVector& particleVector, float qualityThreshold);
	};
}
