#pragma once

#include <ThirdParty/GMapping/sensor/sensor_base/sensor.h>

namespace PTracking
{
	/**
	 * @class BasicSensor
	 * 
	 * @brief Class that implements a sensor.
	 */
	class BasicSensor : public GMapping::Sensor
	{
		public:
			/**
			 * @brief Constructor that takes the name of the sensor as initialization value.
			 * 
			 * It initializes the name of the sensor with the one given in input.
			 * 
			 * @param name sensor name.
			 */
			BasicSensor(const std::string& name = "BasicSensor");
			
			/**
			 * @brief Destructor.
			 */
			~BasicSensor();
	};
}
