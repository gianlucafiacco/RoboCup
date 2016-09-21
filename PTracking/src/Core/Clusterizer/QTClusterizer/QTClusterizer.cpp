#include "QTClusterizer.h"
#include <Utils/Utils.h>

using namespace std;

namespace PTracking
{
	QTClusterizer::QTClusterizer() {;}
	
	QTClusterizer::~QTClusterizer() {;}
	
	void QTClusterizer::clusterize(PoseParticleVector& particleVector, float qualityThreshold)
	{
		vector<vector<pair<PoseParticle,int> > > allCandidateClusters;
		vector<pair<PoseParticle,int> > candidateCluster;
		vector<int> indexParticlesToEliminate;
		PoseParticleVector particles;
		int i, index, j, particlesErased, size;
		
		clusters.clear();
		
		particles = particleVector;
		
		while (particles.size() > 0)
		{
			allCandidateClusters.clear();
			
			for (PoseParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
			{
				index = 0;
				
				candidateCluster.clear();
				
				for (PoseParticleVector::iterator it2 = particles.begin(); it2 != particles.end(); it2++, index++)
				{
					if (isNear(*it,*it2,qualityThreshold))
					{
						candidateCluster.push_back(make_pair(*it2,index));
					}
				}
				
				allCandidateClusters.push_back(candidateCluster);
			}
			
			indexParticlesToEliminate.clear();
			
			const PoseParticleVector& biggestCluster = getBiggestCluster(allCandidateClusters,indexParticlesToEliminate);
			
			clusters.push_back(make_pair(biggestCluster,Utils::calculateCentroid(biggestCluster)));
			
			i = 0;
			j = 0;
			index = indexParticlesToEliminate.front();
			size = indexParticlesToEliminate.size();
			particlesErased = 0;
			
			for (PoseParticleVector::iterator it = particles.begin(); it != particles.end(); )
			{
				if (i != index)
				{
					it++;
					i++;
				}
				else
				{
					it = particles.erase(it);
					
					particlesErased++;
					j++;
					
					if (j < size) index = indexParticlesToEliminate.at(j) - particlesErased;
					else break;
				}
			}
		}
	}
	
	PoseParticleVector QTClusterizer::getBiggestCluster(const vector<vector<pair<PoseParticle,int> > >& allCandidateClusters, vector<int>& indexParticlesToEliminate) const
	{
		PoseParticleVector choosenCluster;
		unsigned int maxSize;
		int i, index;
		
		i = 0;
		index = 0;
		maxSize = 0;
		
		for (vector<vector<pair<PoseParticle,int> > >::const_iterator it = allCandidateClusters.begin(); it != allCandidateClusters.end(); it++, i++)
		{
			if (it->size() > maxSize)
			{
				maxSize = it->size();
				index = i;
			}
		}
		
		const vector<pair<PoseParticle,int> >& candidateCluster = allCandidateClusters.at(index);
		
		for (vector<pair<PoseParticle,int> >::const_iterator it = candidateCluster.begin(); it != candidateCluster.end(); it++)
		{
			choosenCluster.push_back(it->first);
			indexParticlesToEliminate.push_back(it->second);
		}
		
		return choosenCluster;
	}
	
	bool QTClusterizer::isNear(const PoseParticle& p1, const PoseParticle& p2, float qualityThreshold) const
	{
		return ((((p1.pose.pose.x - p2.pose.pose.x) * (p1.pose.pose.x - p2.pose.pose.x)) +
				 ((p1.pose.pose.y - p2.pose.pose.y) * (p1.pose.pose.y - p2.pose.pose.y))) < qualityThreshold);
	}
}
