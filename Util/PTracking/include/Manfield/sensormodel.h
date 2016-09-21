#pragma once

#include <Manfield/prototype_handler.h>
#include <Manfield/filters/gmlocalizer/structs.h>
#include <ThirdParty/GMapping/sensor/sensor_base/sensor.h>
#include <ThirdParty/GMapping/sensor/sensor_base/sensorreading.h>
#include <Utils/Point2f.h>
#include <map>
#include <string>
#include <stdexcept>

namespace manfield
{
	class SensorMap
	{
		public:
			virtual PTracking::Point2f getRandomPointInWorld() const = 0;
	};
	
	class SensorModel
	{
		protected:
			GMapping::Sensor* m_sensor;
			SensorMap* m_sensorMap;
			std::string m_type;
			
		public:
			SensorModel(const std::string& type = "simple") : m_sensor(0), m_sensorMap(0), m_type(type) {;}
			virtual ~SensorModel() {}
			
			virtual SensorModel* clone() const { return 0; }
			virtual void configure(const std::string& filename = "sensormodel.cfg", GMapping::Sensor* sensor = 0) = 0;
			inline const GMapping::Sensor* getSensor() const { return m_sensor; }
			inline const SensorMap* getSensorMap() const { return m_sensorMap; }
			virtual bool likelihood(const GMapping::SensorReading* observation,PoseParticleVector& particles,const LocalizerParameters& params) const { throw std::runtime_error("sensormodel.h::likelihood this function MUST BE redefined."); }
			virtual void sampleFromReading(const GMapping::SensorReading*,PoseParticleVector&,float boosting_factor) const { throw std::runtime_error("sensormodel.h::likelihood this function MUST BE redefined."); }
			inline void setSensorMap(SensorMap* sensorMap) { m_sensorMap = sensorMap; }
			inline void setSensor(GMapping::Sensor* sensor) { m_sensor = sensor; }
			inline std::string toString() const { return m_type; }
			
			#define MODEL_DEFAULT_CLONE(Klass) \
				virtual SensorModel* clone() const { return new Klass(*this); }
	};
	
	typedef std::map<std::string,SensorModel*> SensorModelMap;

	#define SENSORMODEL_FACTORY(Klass) \
		DECLARE_PROTOTYPE(manfield::SensorModel,Klass)
}
