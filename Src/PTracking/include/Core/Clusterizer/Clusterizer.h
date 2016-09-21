#pragma once

#include <Utils/Point2of.h>
#include <Manfield/filters/gmlocalizer/structs.h>
#include <stdexcept>

namespace PTracking
{
	/**
	 * @class Clusterizer
	 * 
	 * @brief Abstract class that defines an interface for a clustering algorithm.
	 */
	class Clusterizer
	{
		protected:
			/**
			 * @brief vector representing the particles and the centroid of each cluster.
			 */
			std::vector<std::pair<PoseParticleVector,Point2of> > clusters;
			
		public:
			/**
			 * @brief Destructor.
			 */
			virtual ~Clusterizer() {;}
			
			/**
			 * @brief Pure virtual function that clusterizes a vector of particles creating a set of clusters with a maximum radius given in input.
			 * 
			 * @param particleVector reference to the particles' vector that have to be clusterized.
			 * @param qualityThreshold maximum radius of a cluster.
			 */
			virtual void clusterize(PoseParticleVector& particleVector, float qualityThreshold) = 0;
			
			/**
			 * @brief Function that returns the vector of clusters.
			 * 
			 * @return the vector of clusters created.
			 */
			inline std::vector<std::pair<PoseParticleVector,Point2of> > getClusters() const { return clusters; }
			
			/**
			 * @brief Function that, if redefined, returns the number of clusters \a k used by the clustering algorithm to clusterize the particles.
			 * 
			 * @return the desired number of clusters \a k. It could throw an exception if the clustering algorithm does not redefine this function (i.e. it is a free-clustering).
			 */
			virtual int getCurrentClusterNumber() const { throw new std::runtime_error("Clusterizer: getCurrentClusterNumber() not redefined!"); }
			
			/**
			 * @brief Function that, if redefined, updates the value of \a k representing the desired number of clusters.
			 * 
			 * @param k new value of the desired clusters.
			 */
			virtual void setMaxClusterNumber(int k) {;}
	};
}
