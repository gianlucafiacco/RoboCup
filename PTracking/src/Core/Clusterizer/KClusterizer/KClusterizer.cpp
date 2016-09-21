#include "KClusterizer.h"
#include <Utils/Utils.h>

using namespace std;

namespace PTracking
{
	KClusterizer::KClusterizer(int maxClusters) : kpoints(maxClusters), maxClusters(maxClusters) {;}
	
	KClusterizer::~KClusterizer() {;}
	
	void KClusterizer::clusterize(PoseParticleVector& particleVector, float qualityThreshold)
	{
		map<float,Cluster> cluster;
		PoseParticleVector particles;
		vector<Point2of> allCentroids;
		vector<int> outliers;
		float distance, temp;
		int clusterIndex, i, j, kint, size;
		
		clusters.clear();
		kpoints.reset();
		
		for (PoseParticleVector::const_iterator it = particleVector.begin(); it != particleVector.end(); it++)
		{
			if (isFarFromAll(*it,qualityThreshold)) kpoints.push(*it);
		}
		
		kint = 0;
		
		for (PriorityBuffer<PoseParticle>::const_iterator cit = kpoints.begin(); cit != kpoints.end(); cit++)
		{
			float index = (cit->second.pose.pose.x * 100000.0) + (cit->second.pose.pose.y * 50000.0) + cit->second.pose.pose.theta;
			
			Cluster c;
			
			c.k = cit->second;
			cluster[index] = c;
		}
		
		i = 0;
		
		for (PoseParticleVector::const_iterator it = particleVector.begin(); it != particleVector.end(); it++, i++)
		{
			int count;
			bool found;
			
			count = 0;
			found = false;
			
			for (PriorityBuffer<PoseParticle>::const_iterator pit = kpoints.begin(); (!found) && (pit != kpoints.end()); pit++)
			{
				if (!isFarFrom(pit->second,*it,qualityThreshold))
				{
					float index = (pit->second.pose.pose.x * 100000.0) + (pit->second.pose.pose.y * 50000.0) + pit->second.pose.pose.theta;
					
					cluster[index].indices.push_back(i);
					found = true;
				}
				
				count++;
			}
			
			if (!found) outliers.push_back(i);
		}
		
		kint = 0;
		
		for (map<float,Cluster>::iterator it = cluster.begin(); it != cluster.end(); it++, kint++)
		{
			float count, cSum, rSum, sSum;
			
			rSum = 0.0;
			cSum = 0.0;
			sSum = 0.0;
			count = 0.0;
			
			particles.clear();
			
			for (vector<int>::iterator vit = it->second.indices.begin(); vit != it->second.indices.end(); vit++, count++)
			{
				PoseParticle& p = particleVector[(*vit)];
				
				rSum += sqrt((p.pose.pose.x * p.pose.pose.x) + (p.pose.pose.y * p.pose.pose.y));
				cSum += cos(p.pose.pose.theta);
				sSum += sin(p.pose.pose.theta);
				
				particles.push_back(p);
			}
			
			if (particles.size() > 0)
			{
				clusters.push_back(make_pair(particles,Utils::calculateCentroid(particles)));
			}
		}
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); it++)
		{
			allCentroids.push_back(it->second);
		}
		
		i = 0;
		size = outliers.size();
		
		// All the outliers will be put in the "right real cluster". The vector outliers represents the index of the outliers and not the elements itself.
		while (i < size)
		{
			const PoseParticle& outlier = particleVector.at(outliers.at(i));
			
			clusterIndex = 0;
			j = 0;
			distance = 1e6;
			
			for (vector<Point2of>::iterator it = allCentroids.begin(); it != allCentroids.end(); it++)
			{
				temp = ((it->x - outlier.pose.pose.x) * (it->x - outlier.pose.pose.x)) + ((it->y - outlier.pose.pose.y) * (it->y - outlier.pose.pose.y));
				
				if (temp < distance)
				{
					clusterIndex = j;
					distance = temp;
				}
				
				j++;
			}
			
			i++;
			
			clusters.at(clusterIndex).first.push_back(outlier);
		}
	}
	
	bool KClusterizer::isFarFrom(const PoseParticle& p1, const PoseParticle& p2, float qualityThreshold) const
	{
		return (sqrt(((p1.pose.pose.x - p2.pose.pose.x) * (p1.pose.pose.x - p2.pose.pose.x)) +
					 ((p1.pose.pose.y - p2.pose.pose.y) * (p1.pose.pose.y - p2.pose.pose.y))) > qualityThreshold);
	}
	
	bool KClusterizer::isFarFromAll(const PoseParticle& p, float qualityThreshold) const
	{
		float distance;
		bool isFarEnough;
		
		distance = 2 * qualityThreshold;
		
		isFarEnough = true;
		
		for (PriorityBuffer<PoseParticle>::const_iterator it = kpoints.begin(); (isFarEnough) && (it != kpoints.end()); it++)
		{
			isFarEnough = isFarFrom(p,it->second,distance);
		}
		
		return isFarEnough;
	}
	
	void KClusterizer::setMaxClusterNumber(int k)
	{
		maxClusters = k;
		
		kpoints = PriorityBuffer<PoseParticle>(maxClusters);
	}
}
