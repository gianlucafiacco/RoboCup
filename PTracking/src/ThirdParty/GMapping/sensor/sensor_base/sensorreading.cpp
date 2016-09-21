#include "sensorreading.h"

namespace GMapping
{
	SensorReading::SensorReading(const Sensor* s, float t) : m_sensor(s), m_time(t) {;}
	
	SensorReading::~SensorReading() {;}
}
