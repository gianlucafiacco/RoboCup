#pragma once

#include "../sensor_base/sensor.h"
#include "../../utils/point.h"
#include <vector>

namespace GMapping
{
	class RangeSensor: public Sensor
	{
		friend class CarmenConfiguration;
		friend class CarmenWrapper;
		friend class Configuration;
			
		public:
			struct Beam
			{
				OrientedPoint pose;
				float span;
				float maxRange;
				float s, c;
			};
			
			bool newFormat;
			
			RangeSensor(std::string name);
			RangeSensor(std::string name, unsigned int beams, float res, const OrientedPoint& position = OrientedPoint(0,0,0), float span = 0,
						float maxrange = 89.0);
			
			inline std::vector<Beam>& beams() { return m_beams; }
			inline const std::vector<Beam>& beams() const { return m_beams; }
			inline OrientedPoint getPose() const { return m_pose; }
			void updateBeamsLookup();
			
		protected:
			std::vector<Beam> m_beams;
			OrientedPoint m_pose;
	};
}
