#include "MultiObjectParticleFilter.h"
#include "../Clusterizer/KClusterizer/KClusterizer.h"
#include "../Clusterizer/QTClusterizer/QTClusterizer.h"
#include "../SensorModels/BasicSensorModel.h"
#include "../Sensors/BasicSensor.h"
#include <Utils/Utils.h>
#include <Manfield/configfile/configfile.h>
#include <algorithm>

using namespace std;
using GMapping::ConfigFile;
using manfield::PrototypeFactory;
using manfield::SensorModel;

namespace PTracking
{
	MultiObjectParticleFilter::MultiObjectParticleFilter(const string& type) : ParticleFilter(type), clusterizer(0) {;}
	
	MultiObjectParticleFilter::~MultiObjectParticleFilter()
	{
		if (clusterizer != 0) delete clusterizer;
		
		for (vector<pair<ParticleFilter*,unsigned long> >::iterator it = particleFilterVector.begin(); it != particleFilterVector.end(); it++)
		{
			if (it->first != 0) delete it->first;
		}
	}
	
	void MultiObjectParticleFilter::checkParticleFilterVectorForCleaning(unsigned long validityTime)
	{
		set<int> filterToDelete;
		int count;
		
		count = 0;
		
		for (vector<pair<ParticleFilter*,unsigned long> >::iterator it = particleFilterVector.begin(); it != particleFilterVector.end(); )
		{
			if ((currentTimestamp - it->second) <= validityTime) it++;
			else
			{
				it = particleFilterVector.erase(it);
				
				filterToDelete.insert(count);
			}
			
			count++;
		}
		
		checkStateMergeMatrixForCleaning(filterToDelete);
		
		if (particleFilterVector.size() == 0)
		{
			init();
		}
	}
	
	void MultiObjectParticleFilter::checkStateMergeMatrixForCleaning(set<int>& filterToDelete)
	{
		int count, count2;
		
		count = 0;
		
		for (vector<vector<pair<int,unsigned long> > >::iterator it = stateMergeMatrix.begin(); it != stateMergeMatrix.end(); )
		{
			/// If the filter have not to be deleted.
			if (filterToDelete.find(count) == filterToDelete.end())
			{
				count2 = 0;
				
				for (vector<pair<int,unsigned long> >::iterator it2 = it->begin(); it2 != it->end(); )
				{
					/// If the filter have not to be deleted.
					if (filterToDelete.find(count2) == filterToDelete.end()) it2++;
					else it2 = it->erase(it2);
					
					count2++;
				}
				
				it++;
			}
			else it = stateMergeMatrix.erase(it);
			
			count++;
		}
	}
	
	void MultiObjectParticleFilter::configure(const string& filename)
	{
		ConfigFile fCfg;
		string key, section;
		float worldXMin = 0, worldXMax = 0, worldYMin = 0, worldYMax = 0;
		
		if (!fCfg.read(filename))
		{
			ERR("Error reading file '" << filename << "' for filter configuration. Exiting..." << endl);
			
			exit(-1);
		}
		
		parametersFile = filename;
		
		ParticleFilter::configure(filename);
		
		string filterName = getname();
		
		SensorModel* sm = 0;
		
		try
		{
			section = "sensormodel";
			key = "type";
			
			sm = PrototypeFactory<SensorModel>::forName(fCfg.value(section,key));
			
			if (sm != 0)
			{
				m_sensorModel = sm;
			}
			else
			{
				ERR("Missing prototype for sensorModel " << string(fCfg.value(section,key)) << ". Exiting..." << endl);
				
				exit(-1);
			}
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		m_sensorModel->configure(filename,new BasicSensor(filterName));
		
		BasicSensorMap* basicSensorMap = new BasicSensorMap();
		
		/// Reading sensor locations.
		section = "location";
		
		try
		{
			key = "worldXMin";
			worldXMin = fCfg.value(section,key);
			
			key = "worldXMax";
			worldXMax = fCfg.value(section,key);
			
			key = "worldYMin";
			worldYMin = fCfg.value(section,key);
			
			key = "worldYMax";
			worldYMax = fCfg.value(section,key);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		basicSensorMap->setworldMin(Point2f(worldXMin,worldYMin));
		basicSensorMap->setworldMax(Point2f(worldXMax,worldYMax));
		
		/// Type of clustering.
		section = "clustering";
		
		try
		{
			key = "algorithm";
			clusteringAlgorithm = string(fCfg.value(section,key));
			
			key = "targetNumber";
			targetNumber = fCfg.value(section,key);
			
			key = "minElementSignificantCluster";
			minElementSignificantCluster = fCfg.value(section,key);
			
			key = "maxTimePassedFromLastOverlap";
			maxTimePassedFromLastOverlap = fCfg.value(section,key);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		thresholdOverlap = 1e6;
		
		if (strcasecmp(clusteringAlgorithm.c_str(),"KClusterizer") == 0) clusterizer = new KClusterizer(targetNumber);
		else if (strcasecmp(clusteringAlgorithm.c_str(),"QTClusterizer") == 0) clusterizer = new QTClusterizer();
		
		Point2f p;
		ifstream ifs;
		string ifsFilename = filename, sensorName;
		char buf[256];
		
		ifs.open(ifsFilename.c_str());
		
		while (ifs.good())
		{
			if (ifs.eof()) break;
			
			ifs.getline(buf,256);
			
			if (string(buf).compare(0,filterName.length(),filterName) == 0)
			{
				istringstream iss(buf);
				
				iss >> sensorName >> p.x >> p.y;
				
				basicSensorMap->insertSensor(p);
			}
		}
		
		ifs.close();
		
		m_sensorModel->setSensorMap(basicSensorMap);
	}
	
	vector<ObjectSensorReading::Observation> MultiObjectParticleFilter::findNearestObservations(ParticleFilter* f, ObjectSensorReading& readings, set<int>& readingAssociated) const
	{
		vector<ObjectSensorReading::Observation> nearestObservations;
		int count;
		
		vector<ObjectSensorReading::Observation>& observations = readings.getObservations();
		
		const Point2f& mean = Utils::calculateMeanParticles(f->getparticles());
		
		count = 0;
		
		for (vector<ObjectSensorReading::Observation>::iterator it = observations.begin(); it != observations.end(); it++)
		{
			if (Utils::isTargetNear(mean,it->observation.getCartesian(),3.0))
			{
				nearestObservations.push_back(*it);
				
				readingAssociated.insert(count);
			}
			
			count++;
		}
		
		return nearestObservations;
	}
	
	vector<pair<PoseParticleVector,Point2of> >& MultiObjectParticleFilter::getClusters()
	{
		clusters.clear();
		
		for (vector<pair<ParticleFilter*,unsigned long> >::const_iterator it = particleFilterVector.begin(); it != particleFilterVector.end(); it++)
		{
			vector<pair<PoseParticleVector,Point2of> > c = it->first->getClusters();
			
			for (vector<pair<PoseParticleVector,Point2of> >::const_iterator it2 = c.begin(); it2 != c.end(); it2++)
			{
				clusters.push_back(*it2);
			}
		}
		
		return clusters;
	}
	
	vector<pair<PoseParticleVector,Point2of> > MultiObjectParticleFilter::getSignificantClusters() const
	{
		vector<pair<PoseParticleVector,Point2of> > significantClusters;
		
		for (vector<pair<ParticleFilter*,unsigned long> >::const_iterator it = particleFilterVector.begin(); it != particleFilterVector.end(); it++)
		{
			const vector<pair<PoseParticleVector,Point2of> >& c = it->first->getClusters();
			
			for (vector<pair<PoseParticleVector,Point2of> >::const_iterator it2 = c.begin(); it2 != c.end(); it2++)
			{
				if (it2->first.size() > minElementSignificantCluster)
				{
					significantClusters.push_back(*it2);
				}
			}
		}
		
		return significantClusters;
	}
	
	void MultiObjectParticleFilter::init()
	{
		vector<pair<int,unsigned long> > state;
		ParticleFilter* f;
		
		f = new ParticleFilter();
		
		f->setSensorModel(this->getSensorModel());
		f->configure(parametersFile);
		f->initFromUniform();
		
		particleFilterVector.push_back(make_pair(f,Timestamp().getMsFromMidnight()));
		
		state.push_back(make_pair(0,0));
		
		stateMergeMatrix.push_back(state);
	}
	
	bool MultiObjectParticleFilter::isThereAlreadyNearFilter(const PolarPoint& observation) const
	{
		const Point2f& ob = observation.getCartesian();
		
		for (vector<pair<ParticleFilter*,unsigned long> >::const_iterator it = particleFilterVector.begin(); it != particleFilterVector.end(); it++)
		{
			const vector<pair<PoseParticleVector,Point2of> >& c = it->first->getClusters();
			
			for (vector<pair<PoseParticleVector,Point2of> >::const_iterator it2 = c.begin(); it2 != c.end(); it2++)
			{
				if (Utils::isTargetNear(ob,it2->second,0.5)) return true;
			}
		}
		
		return false;
	}
	
	void MultiObjectParticleFilter::merge()
	{
		vector<vector<bool> > explored;
		set<int> filterToDelete;
		int clusterIndex1, clusterIndex2, count, filterIndex1, filterIndex2;
		
		for (vector<pair<ParticleFilter*,unsigned long> >::const_iterator it = particleFilterVector.begin(); it != particleFilterVector.end(); it++)
		{
			vector<bool> e;
			
			e.resize(it->first->getClusters().size());
			explored.push_back(e);
		}
		
		filterIndex1 = 0;
		
		for (vector<pair<ParticleFilter*,unsigned long> >::const_iterator it = particleFilterVector.begin(); it != particleFilterVector.end(); it++)
		{
			vector<pair<PoseParticleVector,Point2of> > newCluster;
			
			vector<pair<PoseParticleVector,Point2of> >& c1 = it->first->getClusters();
			
			clusterIndex1 = 0;
			
			for (vector<pair<PoseParticleVector,Point2of> >::const_iterator it2 = c1.begin(); it2 != c1.end(); it2++)
			{
				if (!explored.at(filterIndex1).at(clusterIndex1))
				{
					PoseParticleVector part;
					
					part = it2->first;
					
					explored.at(filterIndex1).at(clusterIndex1) = true;
					
					filterIndex2 = filterIndex1 + 1;
					
					vector<pair<ParticleFilter*,unsigned long> >::const_iterator it3 = it;
					
					it3++;
					
					for ( ; it3 != particleFilterVector.end(); it3++)
					{
						vector<pair<PoseParticleVector,Point2of> > c2 = it3->first->getClusters();
						
						clusterIndex2 = 0;
						
						for (vector<pair<PoseParticleVector,Point2of> >::const_iterator it4 = c2.begin(); it4 != c2.end(); it4++)
						{
							if (!explored.at(filterIndex2).at(clusterIndex2))
							{
								if (Utils::isTargetNear(it2->second,it4->second,1.5))
								{
									explored.at(filterIndex2).at(clusterIndex2) = true;
									
									pair<int,unsigned long>& state = stateMergeMatrix.at(filterIndex1).at(filterIndex2);
									
									if ((currentTimestamp - state.second) < maxTimePassedFromLastOverlap) state.first++;
									else state.first = 1;
									
									state.second = currentTimestamp;
									
									if (state.first > thresholdOverlap) filterToDelete.insert(filterIndex2);
									
									for (PoseParticleVector::const_iterator it5 = it4->first.begin(); it5 != it4->first.end(); it5++) part.push_back(*it5);
								}
							}
							
							clusterIndex2++;
						}
						
						filterIndex2++;
					}
					
					newCluster.push_back(make_pair(part,Utils::calculateCentroid(part)));
				}
				
				clusterIndex1++;
			}
			
			PoseParticleVector part;
			
			for (vector<pair<PoseParticleVector,Point2of> >::iterator it2 = newCluster.begin(); it2 != newCluster.end(); it2++)
			{
				for (PoseParticleVector::iterator it3 = it2->first.begin(); it3 != it2->first.end(); it3++)
				{
					part.push_back(*it3);
				}
			}
			
			/// Sort in decreasing order.
			sort(part.begin(),part.end(),Utils::comparePoseParticle);
			
			if (part.size() > getparticleNumber()) part.resize(getparticleNumber());
			
			it->first->setParticles(part);
			
			filterIndex1++;
		}
		
		count = 0;
		
		for (vector<pair<ParticleFilter*,unsigned long> >::iterator it = particleFilterVector.begin(); it != particleFilterVector.end(); )
		{
			/// If the filter have not to be deleted.
			if (filterToDelete.find(count) == filterToDelete.end())
			{
				it++;
			}
			else
			{
				it = particleFilterVector.erase(it);
			}
			
			count++;
		}
		
		checkStateMergeMatrixForCleaning(filterToDelete);
		
		if (particleFilterVector.size() == 0) init();
	}
	
	void MultiObjectParticleFilter::observe(ObjectSensorReading& readings)
	{
		if (readings.getObservations().size() > 0)
		{
			vector<pair<int,unsigned long> > state;
			set<int> readingAssociated;
			unsigned int size;
			int count, numberOfNewFilters;
			
			size = particleFilterVector.size();
			
			for (unsigned int i = 0; i < size; i++)
			{
				ObjectSensorReading observationsNearestToFilter;
				
				observationsNearestToFilter.setSensor(readings.getSensor());
				
				if (particleFilterVector.size() == 1) observationsNearestToFilter.setObservations(readings.getObservations());
				else
				{
					observationsNearestToFilter.setObservations(findNearestObservations(particleFilterVector.at(i).first,readings,readingAssociated));
				}
				
				if (observationsNearestToFilter.getObservations().size() > 0)
				{
					particleFilterVector.at(i).second = Timestamp().getMsFromMidnight();
					particleFilterVector.at(i).first->observe(&observationsNearestToFilter);
					
					const Point2f& meanParticles = Utils::calculateMeanParticles(particleFilterVector.at(i).first->getparticles());
					const Point2f& sigmaParticles = Utils::calculateSigmaParticles(particleFilterVector.at(i).first->getparticles(),meanParticles);
					
					qualityThreshold = sqrt(1.0 + sigmaParticles.mod()) + 0.02;
					
					clusterizer->clusterize(particleFilterVector.at(i).first->getparticles(),qualityThreshold);
					
					vector<pair<PoseParticleVector,Point2of> > c = clusterizer->getClusters();
					
					if (c.size() > 1) split(c,i);
					else particleFilterVector.at(i).first->setClusters(c);
				}
			}
			
			count = 0;
			numberOfNewFilters = 0;
			
			vector<ObjectSensorReading::Observation>& obs = readings.getObservations();
			
			for (vector<ObjectSensorReading::Observation>::iterator it = obs.begin(); it != obs.end(); it++)
			{
				/// If the reading was not associated to any filter.
				if (readingAssociated.find(count) == readingAssociated.end())
				{
					if (!isThereAlreadyNearFilter(it->observation))
					{
						vector<pair<PoseParticleVector,Point2of> > c;
						PoseParticleVector part;
						ParticleFilter* f;
						
						Point2of temp(it->observation.getCartesian());
						PoseParticle poseParticle;
						
						poseParticle.pose.pose.x = temp.x;
						poseParticle.pose.pose.y = temp.y;
						poseParticle.pose.pose.theta = temp.theta;
						
						fill_n(back_inserter(part),getparticleNumber(),poseParticle);
						
						c.push_back(make_pair(part,static_cast<Point2of>(it->observation.getCartesian())));
						
						f = new ParticleFilter();
						
						f->setSensorModel(this->getSensorModel());
						f->configure(parametersFile);
						f->setParticles(part);
						f->setClusters(c);
						
						particleFilterVector.push_back(make_pair(f,Timestamp().getMsFromMidnight()));
						
						numberOfNewFilters++;
					}
				}
				
				count++;
			}
			
			if (numberOfNewFilters > 0)
			{
				for (vector<vector<pair<int,unsigned long> > >::iterator it = stateMergeMatrix.begin(); it != stateMergeMatrix.end(); it++)
				{
					fill_n(back_inserter(*it),numberOfNewFilters,pair<int,unsigned long>(0,0));
				}
				
				fill_n(back_inserter(state),particleFilterVector.size(),pair<int,unsigned long>(0,0));
				fill_n(back_inserter(stateMergeMatrix),numberOfNewFilters,state);
			}
			
			merge();
			
			clusters.clear();
			
			for (vector<pair<ParticleFilter*,unsigned long> >::iterator it = particleFilterVector.begin(); it != particleFilterVector.end(); it++)
			{
				const vector<pair<PoseParticleVector,Point2of> >& c = it->first->getClusters();
				
				for (vector<pair<PoseParticleVector,Point2of> >::const_iterator it2 = c.begin(); it2 != c.end(); it2++)
				{
					clusters.push_back(*it2);
				}
			}
			
			updateHistoryEstimatedTargets(currentTimestamp,3.0);
		}
	}
	
	void MultiObjectParticleFilter::predict(bool targetSeen, const Timestamp& initialTimestamp, const Timestamp& current)
	{
		PolarPoint trajectory;
		Point2of newEstimatedTarget;
		float dt;
		unsigned int bestParticlesEachFilter;
		
		bestParticlesEachFilter = 0;
		currentTimestamp = current.getMsFromMidnight();
		
		if (particleFilterVector.size() > 0)
		{
			bestParticlesEachFilter = getparticleNumber() / particleFilterVector.size();
			
			m_params.m_particles.clear();
		}
		
		/// Because the timestamps are in milliseconds.
		dt = (static_cast<float>(currentTimestamp - initialTimestamp.getMsFromMidnight()) / 1000.0);
		
		for (vector<pair<ParticleFilter*,unsigned long> >::iterator it = particleFilterVector.begin(); it != particleFilterVector.end(); it++)
		{
			if ((currentTimestamp - lastCheck) > 1e3)
			{
				checkParticleFilterVectorForCleaning(5e3);
				checkHistoryEstimatedTargetForCleaning(currentTimestamp,5e3);
				
				lastCheck = currentTimestamp;
			}
			
			const Point2of& oldEstimatedTarget = static_cast<Point2of>(Utils::calculateMeanParticles(it->first->getparticles()));
			
			if (targetSeen)
			{
				/// Find all the trajectories of estimated targets that are nearest to the centroid of the cluster.
				const vector<PolarPoint>& trajectories = findTrajectoriesEstimatedTargets(oldEstimatedTarget,currentTimestamp,5e3,3.0);
				
				if (trajectories.size() > 0)
				{
					trajectory = trajectories.at(rand() % trajectories.size());
				}
			}
			
			if (trajectory.rho < MAX_FEASIBLE_LINEAR_VELOCITY)
			{
				PointWithVelocity pointWithVelocity;
				
				pointWithVelocity.pose.x = oldEstimatedTarget.x;
				pointWithVelocity.pose.y = oldEstimatedTarget.y;
				pointWithVelocity.pose.theta = oldEstimatedTarget.theta;
				
				const PointWithVelocity& temp = Utils::estimatedPosition(pointWithVelocity,trajectory,dt);
				
				newEstimatedTarget.x = temp.pose.x;
				newEstimatedTarget.y = temp.pose.y;
				newEstimatedTarget.theta = temp.pose.theta;
			}
			else
			{
				newEstimatedTarget = oldEstimatedTarget;
			}
			
			PointWithVelocity oldEstimatedTargetRos, newEstimatedTargetRos;
			
			oldEstimatedTargetRos.pose.x = oldEstimatedTarget.x;
			oldEstimatedTargetRos.pose.y = oldEstimatedTarget.y;
			oldEstimatedTargetRos.pose.theta = oldEstimatedTarget.theta;
			
			newEstimatedTargetRos.pose.x = newEstimatedTarget.x;
			newEstimatedTargetRos.pose.y = newEstimatedTarget.y;
			newEstimatedTargetRos.pose.theta = newEstimatedTarget.theta;
			
			it->first->predict(oldEstimatedTargetRos,newEstimatedTargetRos);
			
			PoseParticleVector& part = it->first->getparticles();
			
			/// Sort in decreasing order.
			sort(part.begin(),part.end(),Utils::comparePoseParticle);
			
			if (bestParticlesEachFilter > part.size())
			{
				bestParticlesEachFilter = part.size();
			}
			
			for (unsigned int i = 0; i < bestParticlesEachFilter; i++)
			{
				m_params.m_particles.push_back(part.at(i));
			}
		}
	}
	
	void MultiObjectParticleFilter::split(vector<pair<PoseParticleVector,Point2of> >& c, unsigned int i)
	{
		vector<pair<int,unsigned long> > state;
		int numberOfNewFilters;
		bool isSplit;
		
		numberOfNewFilters = 0;
		isSplit = false;
		
		const Point2of& p = c.at(0).second;
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = ++c.begin(); it != c.end(); it++)
		{
			if (!Utils::isTargetNear(p,it->second,0.5))
			{
				vector<pair<PoseParticleVector,Point2of> > c2;
				ParticleFilter* f;
				
				c2.push_back(*it);
				
				f = new ParticleFilter();
				
				f->setSensorModel(this->getSensorModel());
				f->configure(parametersFile);
				f->setParticles(it->first);
				f->setClusters(c2);
				
				particleFilterVector.push_back(make_pair(f,Timestamp().getMsFromMidnight()));
				
				isSplit = true;
				numberOfNewFilters++;
			}
		}
		
		if (isSplit)
		{
			vector<pair<PoseParticleVector,Point2of> > c1;
			
			c1.push_back(c.at(0));
			
			particleFilterVector.at(i).first->setParticles(c.at(0).first);
			particleFilterVector.at(i).first->setClusters(c1);
			
			for (vector<vector<pair<int,unsigned long> > >::iterator it = stateMergeMatrix.begin(); it != stateMergeMatrix.end(); it++)
			{
				fill_n(back_inserter(*it),numberOfNewFilters,pair<int,unsigned long>(0,0));
			}
			
			fill_n(back_inserter(state),particleFilterVector.size(),pair<int,unsigned long>(0,0));
			fill_n(back_inserter(stateMergeMatrix),numberOfNewFilters,state);
		}
	}
	
	SENSORFILTER_FACTORY(MultiObjectParticleFilter)
}
