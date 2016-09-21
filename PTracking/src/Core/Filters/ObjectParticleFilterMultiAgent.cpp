#include "ObjectParticleFilterMultiAgent.h"
#include "../Clusterizer/KClusterizer/KClusterizer.h"
#include "../Clusterizer/QTClusterizer/QTClusterizer.h"
#include "../SensorMaps/BasicSensorMap.h"
#include "../Sensors/BasicSensor.h"
#include "../../Utils/Timestamp.h"
#include <Manfield/utils/debugutils.h>
#include <Manfield/configfile/configfile.h>
#include <fstream>

using namespace std;
using GMapping::ConfigFile;
using GMapping::SensorReading;
using manfield::PrototypeFactory;
using manfield::SensorModel;

namespace PTracking
{
	ObjectParticleFilterMultiAgent::ObjectParticleFilterMultiAgent(const string& type) : ParticleFilter(type) {;}
	
	ObjectParticleFilterMultiAgent::~ObjectParticleFilterMultiAgent() {;}
	
	PoseParticle ObjectParticleFilterMultiAgent::adjustWeight(const PoseParticle& p, unsigned long particlesTimestamp, unsigned long currentTimestamp, Utils::DecreaseModelFactor model, float factor) const
	{
		PoseParticle g;
		
		if (model == Utils::Linear)
		{
			g.pose = p.pose;
			
			/// The timestamp is in milliseconds that is why we divide by 1000.0.
			g.weight -= (factor * (((float) (currentTimestamp - particlesTimestamp)) / 1000.0));
			
			if (g.weight < 0.0) g.weight = 0.0;
		}
		
		return g;
	}
	
	void ObjectParticleFilterMultiAgent::configure(const string& filename)
	{
		ConfigFile fCfg;
		string key, section;
		float worldXMin = 0, worldXMax = 0, worldYMin = 0, worldYMax = 0;
		int targetNumber;
		
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
		
		section = "parameters";
		
		try
		{
			key = "bestParticles";
			bestParticlesNumber = fCfg.value(section,key);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		closenessThreshold = 1.0;
		maxAcceptableVariance = std::min((float) 1.0,closenessThreshold * 3);
	}
	
	void ObjectParticleFilterMultiAgent::observe(const vector<ObjectSensorReadingMultiAgent>& readings)
	{
		PoseParticleVector fusedParticles;
		unsigned long currentTimestamp, estimationsTimestamp;
		
		currentTimestamp = Timestamp().getMsFromMidnight();
		
		for (vector<ObjectSensorReadingMultiAgent>::const_iterator it = readings.begin(); it != readings.end(); it++)
		{
			estimationsTimestamp = it->getEstimationsTimestamp();
			
			const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimationsWithtModels = it->getEstimationsWithModels();
			
			for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimationsWithtModels.begin(); it2 != estimationsWithtModels.end(); ++it2)
			{
				const PoseParticleVector& part = Utils::samplingParticles(it2->second.first.observation.getCartesian(),it2->second.first.sigma,bestParticlesNumber);
				
				for (PoseParticleVector::const_iterator it3 = part.begin(); it3 != part.end(); it3++)
				{
					fusedParticles.push_back(adjustWeight(*it3,estimationsTimestamp,currentTimestamp,Utils::Linear,0.15));
				}
			}
		}
		
		m_params.m_particles = fusedParticles;
		
		for (PoseParticleVector::iterator i = m_params.m_particles.begin(); i != m_params.m_particles.end(); i++)
		{
			if (i->weight > m_params.m_maxParticle.weight)
			{
				m_params.m_maxParticle = *i;
			}
		}
		
		updateTargetIdentity(readings);
		
		//clusterizer->clusterize(m_params.m_particles,closenessThreshold);
		//clusters = clusterizer->getClusters();
		
		resample();
	}
	
	void ObjectParticleFilterMultiAgent::resetWeight()
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
	
	void ObjectParticleFilterMultiAgent::updateTargetIdentity(const vector<ObjectSensorReadingMultiAgent>& readings)
	{
		estimationsAgents.clear();
		estimatedTargetModelsWithIdentityMultiAgent.clear();
		
		for (vector<ObjectSensorReadingMultiAgent>::const_iterator it = readings.begin(); it != readings.end(); ++it)
		{
			const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimations = it->getEstimationsWithModels();
			
			for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimations.begin(); it2 != estimations.end(); ++it2)
			{
				estimationsAgents.insert(make_pair(it2->first,it2->second));
			}
		}
		
		pair<multimap<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator,multimap<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator> estimation;
		float allSigmaX, allSigmaY, globalEstimationX, globalEstimationY, sigmaNormalizationRatioX, sigmaNormalizationRatioY;
		int size;
		
		for (multimap<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimationsAgents.begin(); it != estimationsAgents.end(); ++it)
		{
			estimation = estimationsAgents.equal_range(it->first);
			
			if (estimation.first != estimation.second)
			{
				allSigmaX = 0.0;
				allSigmaY = 0.0;
				size = 0;
				
				for (multimap<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimation.first; it2 != estimation.second; ++it2, ++size)
				{
					allSigmaX += it2->second.second.x;
					allSigmaY += it2->second.second.y;
				}
				
				if (size > 1)
				{
					globalEstimationX = 0.0;
					globalEstimationY = 0.0;
					
					sigmaNormalizationRatioX = 0.0;
					sigmaNormalizationRatioY = 0.0;
					
					for (multimap<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimation.first; it2 != estimation.second; ++it2)
					{
						sigmaNormalizationRatioX += (1.0 - (it2->second.second.x / allSigmaX));
						sigmaNormalizationRatioY += (1.0 - (it2->second.second.y / allSigmaY));
					}
					
					sigmaNormalizationRatioX = 1.0 / sigmaNormalizationRatioX;
					sigmaNormalizationRatioY = 1.0 / sigmaNormalizationRatioY;

					for (multimap<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimation.first; it2 != estimation.second; ++it2)
					{
						globalEstimationX += (it2->second.first.observation.getCartesian().x * ((1.0 - (it2->second.second.x / allSigmaX)) * sigmaNormalizationRatioX));
						globalEstimationY += (it2->second.first.observation.getCartesian().y * ((1.0 - (it2->second.second.y / allSigmaY)) * sigmaNormalizationRatioY));
					}
				}
				else
				{
					globalEstimationX = estimation.first->second.first.observation.getCartesian().x;
					globalEstimationY = estimation.first->second.first.observation.getCartesian().y;
				}
				
				ObjectSensorReading::Observation globalEstimation;
				
				globalEstimation.observation.rho = sqrt((globalEstimationX * globalEstimationX) + (globalEstimationY * globalEstimationY));
				globalEstimation.observation.theta = atan2(globalEstimationY,globalEstimationX);
				globalEstimation.sigma = Point2f(allSigmaX / size, allSigmaY / size);
				globalEstimation.model = it->second.first.model;
				
				estimatedTargetModelsWithIdentityMultiAgent.insert(make_pair(it->first,make_pair(globalEstimation,globalEstimation.sigma)));
			}
		}
	}
	
	SENSORFILTER_FACTORY(ObjectParticleFilterMultiAgent)
}
