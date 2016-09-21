#pragma once

#include <map>
#include <string>

namespace GMapping
{
	class Sensor
	{
		protected:
			std::string m_name;
			
		public:
			Sensor(const std::string& name = "");
			virtual ~Sensor();
			
			inline std::string getName() const { return m_name; }
			inline void setName(const std::string& name) { m_name = name; }
	};
	
	typedef std::map<std::string,Sensor*> SensorMap;
}
