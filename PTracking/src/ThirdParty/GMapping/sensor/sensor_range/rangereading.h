#pragma once

#include "rangesensor.h"
#include "../sensor_base/sensorreading.h"
#include <vector>

namespace GMapping
{
	class RangeReading : public SensorReading, public std::vector<float>
	{
		protected:
			OrientedPoint m_pose;
			
		public:
			RangeReading(const RangeSensor* rs, float time = 0);
			RangeReading(unsigned int n_beams, const float* d, const RangeSensor* rs, float time = 0);
			virtual ~RangeReading();
			
			unsigned int activeBeams(float density = 0.) const;
			std::vector<Point> cartesianForm(float maxRange = 1e6) const;
			inline const OrientedPoint& getPose() const { return m_pose; }
			unsigned int rawView(float* v, float density = 0.) const;
			inline void setPose(const OrientedPoint& pose) { m_pose = pose; }
	};
}
