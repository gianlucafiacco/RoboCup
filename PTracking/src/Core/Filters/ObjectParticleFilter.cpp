#include "ObjectParticleFilter.h"
#include "../Clusterizer/KClusterizer/KClusterizer.h"
#include "../Clusterizer/QTClusterizer/QTClusterizer.h"
#include "../SensorModels/BasicSensorModel.h"
#include "../Sensors/BasicSensor.h"
#include <Utils/Utils.h>
#include <Manfield/configfile/configfile.h>
#include <math.h>
#include <algorithm>

using namespace std;
using GMapping::ConfigFile;
using manfield::PrototypeFactory;
using manfield::SensorModel;

namespace PTracking
{
	ObjectParticleFilter::ObjectParticleFilter(const string& type) : ParticleFilter(type), clusterizer(0), maxIdentityNumber(0) {;}
	
	ObjectParticleFilter::~ObjectParticleFilter()
	{
		if (clusterizer != 0) delete clusterizer;
	}
	
	bool ObjectParticleFilter::areModelsSimilar(const ObjectSensorReading::Model& observationModel, const ObjectSensorReading::Model& storedModel)
	{
		return ((abs(observationModel.color[0] - storedModel.color[0]) < 5) &&
				(abs(observationModel.color[1] - storedModel.color[1]) < 5) &&
				(abs(observationModel.color[2] - storedModel.color[2]) < 5));
	}
	
	void ObjectParticleFilter::addParticlesInInterestingPoints(ObjectSensorReading& readings)
	{
		bool associated;
		
		numberOfObservationAssociatedAndPromoted = 0;
		observeNeeded = false;
		
		vector<ObjectSensorReading::Observation>& obs = readings.getObservations();
		
		/// Removing false pending observations.
		for (vector<pair<Point2of,pair<Timestamp,Timestamp> > >::iterator it = pendingObservations.begin(); it != pendingObservations.end(); )
		{
			if ((Timestamp() - it->second.first).getMs() > TIME_TO_WAIT_BEFORE_PROMOTING) it = pendingObservations.erase(it);
			else ++it;
		}
		
		for (vector<ObjectSensorReading::Observation>::iterator it = obs.begin(); it != obs.end(); )
		{
			associated = false;
			
			for (vector<pair<PoseParticleVector,Point2of> >::iterator it2 = clusters.begin(); it2 != clusters.end(); it2++)
			{
				if (Utils::isTargetNear(it->observation.getCartesian(),it2->second,closenessThreshold))
				{
					associated = true;
					observeNeeded = true;
					++numberOfObservationAssociatedAndPromoted;
					
					observationsMapping.push_back(make_pair(it->observation.getCartesian(),it2->second));
					
					break;
				}
			}
			
			if (!associated)
			{
				bool exists;
				
				exists = false;
				
				/// I am looking for a pending observation that can be matched with the current one.
				for (vector<pair<Point2of,pair<Timestamp,Timestamp> > >::iterator it2 = pendingObservations.begin(); it2 != pendingObservations.end(); ++it2)
				{
					if (Utils::isTargetNear(it->observation.getCartesian(),it2->first,closenessThreshold))
					{
						Timestamp temp;
						
						if (((temp - it2->second.first).getMs() < 500.0) && ((temp - it2->second.second).getMs() > TIME_TO_WAIT_BEFORE_PROMOTING))
						{
							associated = true;
							observeNeeded = true;
							pendingObservations.erase(it2);
						}
						else
						{
							exists = true;
							
							it2->first.x = it->observation.getCartesian().x;
							it2->first.y = it->observation.getCartesian().y;
							it2->second.first = temp;
						}
						
						break;
					}
				}
				
				if (associated)
				{
					PoseParticleVector particlesNewPromotedObservation;
					PointWithVelocity pointWithVelocity;
					Point2of temp(it->observation.getCartesian());
					
					pointWithVelocity.pose.x = temp.x;
					pointWithVelocity.pose.y = temp.y;
					pointWithVelocity.pose.theta = temp.theta;
					
					if ((numberOfObservationAssociatedAndPromoted == 0) && (clusters.size() == 0)) m_params.m_particles.clear();
					
					fill_n(back_inserter(particlesNewPromotedObservation),getparticleNumber() * (((numberOfObservationAssociatedAndPromoted == 0) && (clusters.size() == 0)) ? 1.0 : 0.1),
										 PoseParticle(pointWithVelocity,1.0));
					
					for (PoseParticleVector::iterator it = particlesNewPromotedObservation.begin(); it != particlesNewPromotedObservation.end(); ++it)
					{
						/// Adding Gaussian noise.
						Point2of noise(Utils::sampleGaussianSigma(m_params.sr0),Utils::sampleGaussianSigma(m_params.sr0),Utils::sampleGaussianSigma(m_params.st0));
						
						it->pose.pose.x += noise.x;
						it->pose.pose.y += noise.y;
						it->pose.pose.theta = Utils::angNormPiSig(it->pose.pose.theta + noise.theta);
					}
					
					m_params.m_particles.insert(m_params.m_particles.end(),particlesNewPromotedObservation.begin(),particlesNewPromotedObservation.end());
					
					++numberOfObservationAssociatedAndPromoted;
				}
				else if (!exists) pendingObservations.push_back(make_pair(it->observation.getCartesian(),make_pair(Timestamp(),Timestamp())));
			}
			
			if (associated) ++it;
			else it = obs.erase(it);
		}
	}
	
	void ObjectParticleFilter::checkFilterReinitializationNeeded()
	{
		if (clusters.size() == 0) return;
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); it++)
		{
			if (Utils::calculateSigmaParticles(it->first,it->second).mod() < 1.5) return;
		}
		
		/// Re-initialization needed since all the clusters have a high variance.
		pendingObservations.clear();
		clusters.clear();
		estimatedTargetModelsWithIdentity.clear();
		historyEstimatedTargets.clear();
		
		delete clusterizer;
		
		if (strcasecmp(clusteringAlgorithm.c_str(),"KClusterizer") == 0) clusterizer = new KClusterizer(targetNumber);
		else if (strcasecmp(clusteringAlgorithm.c_str(),"QTClusterizer") == 0) clusterizer = new QTClusterizer();
	}
	
	void ObjectParticleFilter::configure(const string& filename)
	{
		ConfigFile fCfg;
		string key, section;
		float worldXMin = 0, worldXMax = 0, worldYMin = 0, worldYMax = 0;
		
		if (!fCfg.read(filename))
		{
			ERR("Error reading file '" << filename << "' for filter configuration. Exiting..." << endl);
			
			exit(-1);
		}
		
		ParticleFilter::configure(filename);
		
		string filterName = getname();
		
		SensorModel* sm = 0;
		
		try
		{
			section = "sensormodel";
			key = "type";
			
			//sm = PrototypeFactory<SensorModel>::forName(fCfg.value(section,key));
			sm = new BasicSensorModel();
			
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
		
		INFO("Map size: [" << (worldXMax - worldXMin) << "," << (worldYMax - worldYMin) << "] meters." << endl);
		
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
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
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
		
		closenessThreshold = 0.7;
		
		INFO("Target closeness threshold: " << closenessThreshold << " meters." << endl);
		
	}
	
	void ObjectParticleFilter::normalizeWeight()
	{
		float totalWeight;
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); it++)
		{
			totalWeight = 0.0;
			
			/// Calculation of total weight of the particles of cluster.
			for (PoseParticleVector::iterator it2 = it->first.begin(); it2 != it->first.end(); it2++)
			{
				totalWeight += it2->weight;
			}
			
			/// Normalize weight of the particles of cluster.
			for (PoseParticleVector::iterator it2 = it->first.begin(); it2 != it->first.end(); it2++)
			{
				it2->weight /= totalWeight;
			}
		}
		
		m_params.m_maxParticle.weight = 0.0;
		
		for (PoseParticleVector::iterator it = m_params.m_particles.begin(); it != m_params.m_particles.end(); it++)
		{
			if (it->weight > m_params.m_maxParticle.weight)
			{
				m_params.m_maxParticle = *it;
			}
		}
	}
	
	void ObjectParticleFilter::observe(ObjectSensorReading& readings)
	{
		ObjectSensorReading readingsCopy;
		
		readingsCopy = readings;
		
		observationsMapping.clear();
		
		checkFilterReinitializationNeeded();
		addParticlesInInterestingPoints(readings);
		
		if (!observeNeeded)
		{
			clusterizer->setMaxClusterNumber(1);
			
			return;
		}
		
		resetWeight();
		
		const BasicSensorModel* basicSensorModel = static_cast<BasicSensorModel*>(getSensorModel());
		
		PointIterator begin = m_params.m_particles.begin();
		PointIterator end = m_params.m_particles.end();
		
		basicSensorModel->likelihood(&readings,begin,end);
		
		/// Sort in decreasing order.
		sort(m_params.m_particles.begin(),m_params.m_particles.end(),Utils::comparePoseParticle);
		
		m_params.m_particles.erase(m_params.m_particles.begin() + getparticleNumber(),m_params.m_particles.end());
		
		/// Adapting target's number.
		if (numberOfObservationAssociatedAndPromoted > 0)
		{
			if (numberOfObservationAssociatedAndPromoted >= clusters.size())
			{
				clusterizer->setMaxClusterNumber(numberOfObservationAssociatedAndPromoted);
				
				lastTimeShouldBeDecreased.setToNow();
			}
			else
			{
				if ((Timestamp() - lastTimeShouldBeDecreased).getMs() > TIME_TO_WAIT_BEFORE_PROMOTING)
				{
					clusterizer->setMaxClusterNumber((clusterizer->getCurrentClusterNumber() > 1) ? (clusterizer->getCurrentClusterNumber() - 1) : 1);
				}
			}
		}
		
		clusterizer->clusterize(m_params.m_particles,closenessThreshold);
		clusters = clusterizer->getClusters();
		
		updateTargetIdentity(readingsCopy);
		
		normalizeWeight();
		updateHistoryEstimatedTargets(currentTimestamp,closenessThreshold);
		resample();
	}
	
	void ObjectParticleFilter::predict(const Point2of& newRobotPose, const Point2of& oldRobotPose, bool targetSeen, const Timestamp& initialTimestamp, const Timestamp& current)
	{
		PolarPoint trajectory;
		float a, b, c, d, deltaX, deltaY, deltaTheta, deltaTheta2, dt;
		unsigned int bestParticlesEachCluster, i;
		
		bestParticlesEachCluster = 0;
		currentTimestamp = current.getMsFromMidnight();
		
		if ((currentTimestamp - lastCheck) > 1e3)
		{
			checkHistoryEstimatedTargetForCleaning(currentTimestamp,5e3);
			
			lastCheck = currentTimestamp;
		}
		
		if (clusters.size() > 0)
		{
			bestParticlesEachCluster = getparticleNumber() / clusters.size();
			
			m_params.m_particles.clear();
		}
		
		/// Because the timestamps are in milliseconds.
		dt = (static_cast<float>(currentTimestamp - initialTimestamp.getMsFromMidnight()) / 1000.0);
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); it++)
		{
			if (targetSeen)
			{
				/// Find all the trajectories of estimated targets that are nearest to the centroid of the cluster.
				const vector<PolarPoint>& trajectories = findTrajectoriesEstimatedTargets(it->second,currentTimestamp,5e3,3.0);
				
				if (trajectories.size() > 0) trajectory = trajectories.at(rand() % trajectories.size());
			}
			
			PoseParticleVector& part = it->first;
			
			if (bestParticlesEachCluster > part.size()) bestParticlesEachCluster = part.size();
			
			i = 0;
			
			for (PoseParticleVector::iterator it2 = part.begin(); it2 != part.end(); it2++, i++)
			{
				if (i > bestParticlesEachCluster) break;
				
				if (trajectory.rho < FEASIBLE_LINEAR_VELOCITY)
				{
					const PointWithVelocity& newPose = Utils::estimatedPosition(it2->pose,trajectory,dt);
					
					it2->pose.pose.x = newPose.pose.x;
					it2->pose.pose.y = newPose.pose.y;
				}
				
				/// Adding Gaussian noise.
				Point2of noise(Utils::sampleGaussianSigma(m_params.sr0),Utils::sampleGaussianSigma(m_params.sr0),Utils::sampleGaussianSigma(m_params.st0));
				
				it2->pose.pose.x += noise.x;
				it2->pose.pose.y += noise.y;
				it2->pose.pose.theta = Utils::angNormPiSig(it2->pose.pose.theta + noise.theta);
				
				m_params.m_particles.push_back(*it2);
			}
		}
		
		if (m_params.m_particles.size() < getparticleNumber())
		{
			unsigned int difference;
			
			difference = getparticleNumber() - m_params.m_particles.size();
			
			for (unsigned int j = 0; j < difference; j++) m_params.m_particles.push_back(m_params.m_particles.at(j));
		}
		
		/// I am considering the robot's movement too. So, the particles are moved coherently.
		deltaX = cos(-newRobotPose.theta) * (newRobotPose.x - oldRobotPose.x) - sin(-newRobotPose.theta) * (newRobotPose.y - oldRobotPose.y);
		deltaY = sin(-newRobotPose.theta) * (newRobotPose.x - oldRobotPose.x) + cos(-newRobotPose.theta) * (newRobotPose.y - oldRobotPose.y);
		deltaTheta = Utils::angNormPiSig(newRobotPose.theta - oldRobotPose.theta);
		deltaTheta2 = deltaTheta / 2;
		
		a = cos(oldRobotPose.theta + deltaTheta2);
		b = cos(M_PI / 2 + oldRobotPose.theta + deltaTheta2);
		
		c = sin(oldRobotPose.theta + deltaTheta2);
		d = sin(M_PI / 2 + oldRobotPose.theta + deltaTheta2);
		
		for (PoseParticleVector::iterator it = m_params.m_particles.begin(); it != m_params.m_particles.end(); ++it)
		{
			/// Runge-Kutta approximation.
			it->pose.pose.x += (deltaX * a) + (deltaY * b);
			it->pose.pose.y += (deltaX * c) + (deltaY * d);
			it->pose.pose.theta = Utils::angNormPiSig(it->pose.pose.theta + deltaTheta);
		}
		
		for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it = estimatedTargetModelsWithIdentity.begin(); it != estimatedTargetModelsWithIdentity.end(); ++it)
		{
			it->second.first.sigma.x += 0.01;
			it->second.first.sigma.y += 0.01;
			it->second.second = it->second.first.sigma;
		}
	}
	
	void ObjectParticleFilter::resample()
	{
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
		{
			vector<int> indexes;
			
			indexes.reserve(2 * it->first.size());
			::resample(indexes,it->first);
			repeatIndexes(it->first,indexes,it->first);
		}
	}
	
	void ObjectParticleFilter::resetWeight()
	{
		int i, size;
		
		i = 0;
		size = m_params.m_particles.size();
		
		/// Partial Loop Unrolling to better use pipeling.
		for (; i < size - 3; i += 4)
		{
			m_params.m_particles.at(i).weight = 1.0;
			m_params.m_particles.at(i + 1).weight = 1.0;
			m_params.m_particles.at(i + 2).weight = 1.0;
			m_params.m_particles.at(i + 3).weight = 1.0;
		}
		
		for (; i < size; ++i)
		{
			m_params.m_particles.at(i).weight = 1.0;
		}
	}
	
	void ObjectParticleFilter::updateTargetIdentity(ObjectSensorReading& readings)
	{
		vector<int> estimationsValid;
		int i;
		
		vector<ObjectSensorReading::Observation>& obs = readings.getObservations();
		
		for (vector<pair<PoseParticleVector,Point2of> >::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
		{
			float distance, minDistance;
			int index;
			
			minDistance = FLT_MAX;
			index = -1;
			
			for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimatedTargetModelsWithIdentity.begin(); it2 != estimatedTargetModelsWithIdentity.end(); ++it2)
			{
				const vector<int>::const_iterator& e = find(estimationsValid.begin(),estimationsValid.end(),it2->first);
				
				/// The estimation has been already associated to a cluster.
				if (e != estimationsValid.end()) continue;
				
				const Point2f& estimation = it2->second.first.observation.getCartesian();
				
				distance = sqrt(((it->second.x - estimation.x) * (it->second.x - estimation.x)) + ((it->second.y - estimation.y) * (it->second.y - estimation.y)));
				
				if (distance < minDistance)
				{
					index = it2->first;
					minDistance = distance;
				}
			}
			
			/// Updating target with identity.
			if (minDistance < closenessThreshold)
			{
				const map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator& estimation = estimatedTargetModelsWithIdentity.find(index);
				
				estimation->second.first.observation.rho = it->second.mod();
				estimation->second.first.observation.theta = atan2(it->second.y,it->second.x);
				estimation->second.first.sigma = Utils::calculateSigmaParticles(it->first,it->second);
				estimation->second.second = estimation->second.first.sigma;
				
				estimationsValid.push_back(index);
				
				const map<int,Timestamp>::iterator& estimationTime = estimationsUpdateTime.find(index);
				
				/// Updating the timestamp of the estimation.
				estimationTime->second.setToNow();
			}
			/// Searching for an existing model of the estimation.
			else
			{
				minDistance = FLT_MAX;
				index = -1;
				i = 0;
				
				for (vector<ObjectSensorReading::Observation>::iterator it2 = obs.begin(); it2 != obs.end(); ++it2, ++i)
				{
					const Point2f& o = it2->observation.getCartesian();
					
					distance = ((it->second.x - o.x) * (it->second.x - o.x)) + ((it->second.y - o.y) * (it->second.y - o.y));
					
					if (distance < minDistance)
					{
						minDistance = distance;
						index = i;
					}
				}
				
				/// An observation close to the cluster has been found.
				if (minDistance < closenessThreshold)
				{
					ObjectSensorReading::Observation target;
					ObjectSensorReading::Model model;
					int targetIdentity;
					
					model = obs.at(index).model;
					
					/// Observation used and no longer needed.
					obs.erase(obs.begin() + index);
					
					index = -1;
					i = 0;
					
					for (vector<pair<ObjectSensorReading::Model,int> >::const_iterator it2 = targetModels.begin(); it2 != targetModels.end(); ++it2, ++i)
					{
						if (areModelsSimilar(model,it2->first))
						{
							index = i;
							
							break;
						}
					}
					
					target.observation = PolarPoint(atan2(it->second.y,it->second.x),it->second.mod());
					target.sigma = Utils::calculateSigmaParticles(it->first,it->second);
					
					/// An existing model has been found.
					if (index != -1)
					{
						target.model = targetModels.at(index).first;
						targetIdentity = targetModels.at(index).second;
					}
					else
					{
						target.model = model;
						targetIdentity = ++maxIdentityNumber;
						
						targetModels.push_back(make_pair(target.model,targetIdentity));
					}
					
					estimatedTargetModelsWithIdentity.insert(make_pair(targetIdentity,make_pair(target,target.sigma)));
					estimationsUpdateTime.insert(make_pair(targetIdentity,Timestamp()));
					
					estimationsValid.push_back(targetIdentity);
				}
			}
		}
		
		/// Removing all the estimations that have not been associated in this iteration.
		for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it = estimatedTargetModelsWithIdentity.begin(); it != estimatedTargetModelsWithIdentity.end(); )
		{
			const vector<int>::const_iterator& e = find(estimationsValid.begin(),estimationsValid.end(),it->first);
			
			if (e != estimationsValid.end()) ++it;
			else
			{
				const map<int,Timestamp>::iterator& estimationTime = estimationsUpdateTime.find(it->first);
				
				if ((currentTimestamp - estimationTime->second.getMsFromMidnight()) > TIME_TO_WAIT_BEFORE_DELETING) estimatedTargetModelsWithIdentity.erase(it++);
				else ++it;
			}
		}
	}
	
	SENSORFILTER_FACTORY(ObjectParticleFilter)
}
