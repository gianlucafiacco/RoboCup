#pragma once

#include "sensor.h"

namespace GMapping
{
	class SensorReading
	{
		protected:
			const Sensor* m_sensor;
			float m_time;
			
		public:
			SensorReading(const Sensor* s = 0, float time = 0);
			virtual ~SensorReading();
			
			inline const Sensor* getSensor() const { return m_sensor; }
			inline float getTime() const { return m_time; }
			inline void setTime(float t) { m_time = t; }
	};
}
