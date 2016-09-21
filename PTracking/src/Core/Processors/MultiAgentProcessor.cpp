#include "MultiAgentProcessor.h"
#include "Core/Clusterizer/Clusterizer.h"
#include <Manfield/configfile/configfile.h>
#include <Manfield/utils/debugutils.h>

using namespace std;
using GMapping::ConfigFile;
using GMapping::SensorReading;

namespace PTracking
{
	// In seconds.
	static const float UPDATE_FREQUENCY = 1;
	
	MultiAgentProcessor::MultiAgentProcessor() : ManifoldFilterProcessor(), m_updateFrequency(UPDATE_FREQUENCY), m_nFusedParticles(0) {;}
	
	MultiAgentProcessor::~MultiAgentProcessor() {;}
	
	void MultiAgentProcessor::init()
	{
		ManifoldFilterProcessor::init();
	}
	
	void MultiAgentProcessor::processReading(const vector<ObjectSensorReadingMultiAgent>& readings)
	{
		if (m_first)
		{
			m_first = false;
			m_bootstrapRequired = true;
		}
		
		if (updateNeeded())
		{
			updateBootStrap();
			
			for (FilterBank::const_iterator it = m_filterBank.begin(); it != m_filterBank.end(); it++)
			{
				singleFilterIteration(*static_cast<ObjectParticleFilterMultiAgent*>(it->second),readings);
			}
			
			timeOfLastIteration.setToNow();
		}
	}
	
	void MultiAgentProcessor::singleFilterIteration(ObjectParticleFilterMultiAgent& f, const vector<ObjectSensorReadingMultiAgent>& readings) const
	{
		if (readings.size() > 0) f.observe(readings);
	}
	
	bool MultiAgentProcessor::updateNeeded() const
	{
		return true;
	}
}
