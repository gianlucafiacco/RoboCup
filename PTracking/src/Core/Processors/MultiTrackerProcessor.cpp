#include "MultiTrackerProcessor.h"
#include "../Filters/ObjectSensorReading.h"
#include <Utils/Utils.h>
#include <Manfield/utils/debugutils.h>

using namespace std;
using GMapping::SensorReading;

namespace PTracking
{
	// In seconds.
	static const float UPDATE_FREQUENCY = 1;
	
	MultiTrackerProcessor::MultiTrackerProcessor() : ManifoldFilterProcessor(), m_updateFrequency(UPDATE_FREQUENCY), m_nFusedParticles(0) {;}
	
	MultiTrackerProcessor::~MultiTrackerProcessor() {;}
	
	void MultiTrackerProcessor::processReading(const Point2of& robotPose, bool targetSeen, const Timestamp& initialTimestamp, const Timestamp& currentTimestamp, vector<ObjectSensorReading>& readings)
	{
		if (m_first)
		{
			m_first = false;
			m_bootstrapRequired = true;
		}
		
		if (updateNeeded(robotPose,currentTimestamp))
		{
			updateBootStrap();
			
			for (FilterBank::const_iterator it = m_filterBank.begin(); it != m_filterBank.end(); it++)
			{
				singleFilterIteration(*static_cast<MultiObjectParticleFilter*>(it->second),targetSeen,initialTimestamp,currentTimestamp,readings);
			}
			
			// Resetting clock.
			lastRobotPose = robotPose;
			timeOfLastIteration.setToNow();
		}
	}
	
	void MultiTrackerProcessor::singleFilterIteration(MultiObjectParticleFilter& f, bool targetSeen, const Timestamp& initialTimestamp, const Timestamp& currentTimestamp,
													  vector<ObjectSensorReading>& readings) const
	{
		f.predict(targetSeen,initialTimestamp,currentTimestamp);
		
		for (vector<ObjectSensorReading>::iterator it = readings.begin(); it != readings.end(); it++)
		{
			if (it->getSensor().getName() == f.getSensor().getName())
			{
				if (it->getObservations().size() > 0)
				{
					f.observe(*it);
				}
			}
		}
	}
	
	bool MultiTrackerProcessor::updateNeeded(const Point2of& robotPose, const Timestamp& currentTimestamp) const
	{
		if ((currentTimestamp - timeOfLastIteration).getMs() > MAX_TIME_TO_WAIT) return true;
		
		if (fabs(robotPose.mod() - lastRobotPose.mod()) > 0.1) return true;
		
		if (Utils::rad2deg(fabs(robotPose.theta - lastRobotPose.theta)) > 5.0) return true;
		
		return false;
	}
}
