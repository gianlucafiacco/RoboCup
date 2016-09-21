#pragma once

#include "sensorfilter.h"
#include <vector>

namespace manfield
{
	class ManifoldFilterProcessor
	{
		protected:
			typedef std::map<std::string, SensorFilter*> FilterBank;
			
			FilterBank m_filterBank;
			PTracking::PointWithVelocity m_lastOdometryPose;
			float m_timeOfLastIteration;
			bool m_bootstrapRequired, m_first, ownFilterBank;
			
			virtual void singleFilterIteration(SensorFilter* sf, const PTracking::PointWithVelocity& odometry,
											   const std::vector<GMapping::SensorReading*>& reading);
			
		public:
			ManifoldFilterProcessor (bool ownFB = false);
			virtual ~ManifoldFilterProcessor();
			
			virtual void addSensorFilter(SensorFilter* filter);
			const FilterBank& getFilterBank() const { return m_filterBank; }
			void init();
			void initFromReadings(const std::vector<GMapping::SensorReading*>& reading);
			void initFromUniform();
			virtual void processReading(const PTracking::PointWithVelocity& odometry, const std::vector<GMapping::SensorReading*>& reading);
			virtual void updateBootStrap();
			virtual bool updateNeeded() { return true; }
	};
}
