#include "localizer.h"
#include <Utils/Utils.h>
#include <string>

using namespace std;
using PTracking::Point2f;
using PTracking::PointWithVelocity;
using PTracking::Utils;

const float LAMBDA_DECAY_TIME_DECELERATION = 1; // sec
const float LAMBDA_DECELERATION            = LAMBDA_DECAY_TIME_DECELERATION / 0.385662481;

Localizer::Localizer(unsigned int particles, unsigned int pointSkip)
{
	m_params.m_pointSkip = pointSkip;
	m_params.m_particleNumber = particles;
	m_params.m_particles.resize(particles);
}

bool Localizer::initFromMap(const manfield::SensorModelMap& smap)
{
	if (smap.empty()) return false;
	
	size_t interval = m_params.m_particles.size() / smap.size();
	size_t currentIndex = 0;
	
	for (manfield::SensorModelMap::const_iterator itsm = smap.begin(); itsm != smap.end(); ++itsm)
	{
		const manfield::SensorMap* sm = itsm->second->getSensorMap();
		
		for (size_t particleIndex = 0; sm && (currentIndex < m_params.m_particles.size()) && (particleIndex <= interval); ++currentIndex, ++particleIndex)
		{
			Point2f mpose = sm->getRandomPointInWorld();
			
			m_params.m_particles[currentIndex].pose.pose.x = mpose.x;
			m_params.m_particles[currentIndex].pose.pose.y = mpose.y;
			m_params.m_particles[currentIndex].pose.pose.theta = Utils::randr(M_PI);
			m_params.m_particles[currentIndex].weight = 1.;
		}
	}
	
	return true;
}

bool Localizer::initFromMap(const manfield::SensorModel& smodel)
{
	const manfield::SensorMap* sm = smodel.getSensorMap();
	
	for (PoseParticleVector::iterator it = m_params.m_particles.begin(); sm && (it != m_params.m_particles.end()); ++it)
	{
		Point2f mpose = sm->getRandomPointInWorld();
		
		it->pose.pose.x = mpose.x;
		it->pose.pose.y = mpose.y;
		it->pose.pose.theta = Utils::randr(M_PI);
		it->weight = 1.0;
	}
	
	return true;
}

bool Localizer::initFromReadings(const manfield::SensorModelMap&, const vector<GMapping::SensorReading*>&)
{
	cerr << "Error: Localizer::initFromReadings(const manfield::SensorModelMap&, const vector<GMapping::SensorReading*>&) not implemented yet." << endl;
	
	return false;
}

bool Localizer::initFromReadings(const manfield::SensorModel&, const vector<GMapping::SensorReading*>&)
{
	cerr << "Localizer::initFromReadings(const manfield::SensorModel&, const vector<GMapping::SensorReading*>&) not implemented yet." << endl;
	
	return false;
}

void Localizer::initFromSamples(const PoseParticleVector& particles)
{
	vector<int> indexes;
	
	::resample(indexes,particles,m_params.m_particles.size());
	repeatIndexes(m_params.m_particles,indexes,particles);
}

void Localizer::initUniform(float xmin, float xmax, float ymin, float ymax)
{
	for (PoseParticleVector::iterator it = m_params.m_particles.begin(); it != m_params.m_particles.end(); ++it)
	{
		it->pose.pose.x = (float) (rand()) / (float) RAND_MAX * (xmax - xmin) + xmin;
		it->pose.pose.y = (float) (rand()) / (float) RAND_MAX * (ymax - ymin) + ymin;
		it->pose.pose.theta = (float) (rand()) / (float) RAND_MAX * (2 * M_PI) - M_PI;
		it->weight = 1.0;
	}
}

bool Localizer::observe(const manfield::SensorModel& model, const GMapping::SensorReading* observation)
{
	if (model.likelihood(observation,m_params.m_particles, m_params))
	{
		m_params.m_maxParticle.weight = 0;
		
		for (PoseParticleVector::iterator it = m_params.m_particles.begin(); it != m_params.m_particles.end(); ++it)
		{
			if (it->weight > m_params.m_maxParticle.weight) m_params.m_maxParticle = *it;
		}
		
		return true;
	}
	
	return false;
}

void Localizer::resample()
{
	vector<int> indexes;
	
	::resample(indexes,m_params.m_particles);
	
	PoseParticleVector newParticles;
	
	repeatIndexes(newParticles,indexes,m_params.m_particles);
	
	m_params.m_particles = newParticles;
}

void Localizer::resetWeight()
{
	for (PoseParticleVector::iterator it = m_params.m_particles.begin(); it != m_params.m_particles.end(); ++it) it->weight = 1.0;
}

void Localizer::sensorResetFromReadings(const manfield::SensorModelMap&, const vector<GMapping::SensorReading*>&, float) {;}
