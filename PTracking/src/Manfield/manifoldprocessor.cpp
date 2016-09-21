#include "manifoldprocessor.h"
#include <ext/functional>
#include <algorithm>

using namespace std;
using PTracking::PointWithVelocity;

namespace manfield
{
	ManifoldFilterProcessor::ManifoldFilterProcessor(bool ownFB) : m_bootstrapRequired(false), m_first(true), ownFilterBank(ownFB) {;}
	
	ManifoldFilterProcessor::~ManifoldFilterProcessor()
	{
		if (ownFilterBank)
		{
			for (FilterBank::iterator it = m_filterBank.begin(); it != m_filterBank.end(); ++it) delete it->second;
		}
		
		m_filterBank.clear();
	}
	
	void ManifoldFilterProcessor::addSensorFilter(SensorFilter* filter)
	{
		FilterBank::iterator sf = m_filterBank.find(filter->gettype());
		
		if (sf != m_filterBank.end())
		{
			delete sf->second;
			
			m_filterBank.erase(sf);
		}
		
		m_filterBank.insert(make_pair(filter->getname(),filter));
	}
	
	void ManifoldFilterProcessor::init()
	{
		m_first = true;
	}
	
	void ManifoldFilterProcessor::initFromReadings(const vector<GMapping::SensorReading*>& readings)
	{
		for (FilterBank::const_iterator it = m_filterBank.begin(); it != m_filterBank.end(); ++it)
		{
			it->second->initFromReadings(readings);
		}
	}
	
	void ManifoldFilterProcessor::initFromUniform()
	{
		for (FilterBank::const_iterator it = m_filterBank.begin(); it != m_filterBank.end(); ++it)
		{
			it->second->initFromUniform();
		}
	}
	
	void ManifoldFilterProcessor::processReading(const PointWithVelocity& odometry, const vector<GMapping::SensorReading*>& reading)
	{
		cerr << "Using the default implementation of ManifoldFilterProcessor::processReading, please verify that is what you need." << endl;
		
		if (m_first)
		{
			m_first = false;
			m_bootstrapRequired = true;
		}
		
		if (updateNeeded())
		{
			updateBootStrap();
			
			for (FilterBank::const_iterator it = m_filterBank.begin(); it != m_filterBank.end(); ++it)
			{
				singleFilterIteration(it->second, odometry, reading);
			}
			
			m_lastOdometryPose = odometry;
		}
	}
	
	void ManifoldFilterProcessor::singleFilterIteration(SensorFilter* sf, const PointWithVelocity& odometry,
														const vector<GMapping::SensorReading*>& reading)
	{
		sf->predict(m_lastOdometryPose,odometry);
		
		for (vector<GMapping::SensorReading*>::const_iterator it = reading.begin(); it != reading.end(); ++it)
		{
			if ((*it)->getSensor()->getName() == sf->getSensor()->getName())
			{
				sf->observe(*it);
			}
		}
	}
	
	void ManifoldFilterProcessor::updateBootStrap()
	{
		if (m_bootstrapRequired)
		{
			initFromUniform();
			
			m_bootstrapRequired = false;
		}
	}
}
