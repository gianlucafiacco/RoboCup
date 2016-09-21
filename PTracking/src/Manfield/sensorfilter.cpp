#include <Manfield/sensorfilter.h>

using namespace std;

namespace manfield
{
	SensorFilter::SensorFilter(const string& type, const string& name, bool ownSM) : m_sensorModel(0), ownSensorModel(ownSM), m_type(type),
																							   m_name(name) {;}
	
	SensorFilter::~SensorFilter()
	{
		if (ownSensorModel)
		{
			if (m_sensorModel != 0) delete m_sensorModel;
			
			m_sensorModel = 0;
		}
	}
}
