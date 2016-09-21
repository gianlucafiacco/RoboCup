#pragma once

#include <Manfield/prototype_handler.h>
#include <Manfield/sensormodel.h>
#include <ThirdParty/GMapping/sensor/sensor_base/sensorreading.h>
#include <ThirdParty/GMapping/utils/macro_params.h>
#include <map>
#include <string>
#include <vector>

namespace manfield
{
	class SensorFilter
	{
		protected:
			SensorModel* m_sensorModel;
			bool ownSensorModel;
			
		public:
			SensorFilter(const std::string& type = "undefined", const std::string& name = "unnamed", bool ownSM = false);
			virtual ~SensorFilter();
			
			virtual SensorFilter* clone() const { return 0; }
			virtual void configure(const std::string& filename) = 0;
			const GMapping::Sensor* getSensor() const { return m_sensorModel->getSensor(); }
			SensorModel* getSensorModel() { return m_sensorModel; }
			virtual void initFromReadings(const std::vector<GMapping::SensorReading*>&) {}
			virtual void initFromUniform() = 0;
			virtual void observe(const GMapping::SensorReading* reading) = 0;
			virtual void predict(const PTracking::PointWithVelocity& oldOdometry, const PTracking::PointWithVelocity& newOdometry) = 0;
			void setSensorModel(SensorModel* sm) { m_sensorModel = sm; }
			inline std::string toString() const { return m_type; }
			
			#define FILTER_DEFAULT_CLONE(Klass) \
				virtual SensorFilter* clone() const { return new Klass(*this); }
			
			PARAM_GET(std::string,type,private,public)
			PARAM_SET_GET(std::string,name,private,public,public)
	};
	
	#define SENSORFILTER_FACTORY(Klass) \
		DECLARE_PROTOTYPE(manfield::SensorFilter, Klass)
}
