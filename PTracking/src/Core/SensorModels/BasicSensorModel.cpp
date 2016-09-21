#include "BasicSensorModel.h"
#include <Manfield/configfile/configfile.h>
#include <Manfield/utils/debugutils.h>
#include <stdlib.h>

using namespace std;
using namespace GMapping;

namespace PTracking
{
	BasicSensorModel::BasicSensorModel(const string& type) : SensorModel(type), linearVelocity(defaultLinearVelocity) {;}
	
	BasicSensorModel::~BasicSensorModel() {;}
	
	void BasicSensorModel::calculateLinearVelocity(unsigned long initialTimestamp, unsigned long currentTimestamp)
	{
		linearVelocity = defaultLinearVelocity * exp(-((float) (currentTimestamp - initialTimestamp)));
	}
	
	void BasicSensorModel::configure(const string& filename, GMapping::Sensor* sensor)
	{
		ConfigFile fCfg;
		string key, section;
		
		SensorModel::setSensor(sensor);
		
		if (!fCfg.read(filename))
		{
			ERR("Error reading file '" << filename << "' for sensor model configuration. Exiting..." << endl);
			
			exit(-1);
		}
		
		section = "sensormodel";
		
		try
		{
			key = "defaultLinearVelocity";
			defaultLinearVelocity = fCfg.value(section,key);
			
			key = "sigmaRho";
			sigmaRho = fCfg.value(section,key);
			
			key = "sigmaTheta";
			sigmaTheta = fCfg.value(section,key);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
	}
	
	void BasicSensorModel::likelihood(SensorReading* reading, PointIterator& particlesBegin, PointIterator& particlesEnd) const
	{
		ObjectSensorReading* osr = dynamic_cast<ObjectSensorReading*>(reading);
		
		if (osr == 0) return;
		
		BasicSensorMap* map = dynamic_cast<BasicSensorMap*>(m_sensorMap);
		
		if (map == 0) return;
		
		for (PoseParticleVector::iterator i = particlesBegin; i != particlesEnd; i++)
		{
			// We assume that the weight is resetted at the beginning of the new iteration.
			i->weight *= likelihood(map,i->pose,osr->getObservations());
		}
	}
	
	float BasicSensorModel::likelihood(BasicSensorMap* map, PointWithVelocity& pose, vector<ObjectSensorReading::Observation>& observations) const
	{
		if (!map->isInsideWorld(pose.pose.x,pose.pose.y)) return 0.0;
		
		float deltaRho, l;
		
		l = 0.0;
		
		for (vector<ObjectSensorReading::Observation>::iterator it = observations.begin(); it != observations.end(); it++)
		{
			deltaRho = sqrt(((pose.pose.x - it->observation.getCartesian().x) * (pose.pose.x - it->observation.getCartesian().x)) +
							((pose.pose.y - it->observation.getCartesian().y) * (pose.pose.y - it->observation.getCartesian().y)));
			
			l += exp(-deltaRho / sigmaRho);
		}
		
		return l;
	}
	
	SENSORMODEL_FACTORY(BasicSensorModel)
}
