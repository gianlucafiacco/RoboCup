#pragma once

#include <Manfield/filters/gmlocalizer/localizer.h>
#include <Manfield/sensorfilter.h>
#include <Core/Filters/ObjectSensorReading.h>
#include <Utils/Point2f.h>
#include <Utils/Point2of.h>
#include <Utils/PolarPoint.h>
#include <Utils/PriorityBuffer.h>

namespace manfield
{
	class ParticleFilter : public SensorFilter, public Localizer
	{
		private:
			static const unsigned int MAX_ELEMENTS_PRIORITY_BUFFER = 30;
			
		protected:
			struct PairPriorityBuffer
			{
				PTracking::Point2of estimatedTarget;
				// Note: the weight represents the timestamp of the estimated target but in order to use the class PriorityBuffer the name must be weight.
				unsigned long weight;
			};
			
			std::vector<std::pair<PoseParticleVector,PTracking::Point2of> > clusters;
			std::list<PTracking::PriorityBuffer<PairPriorityBuffer>*> historyEstimatedTargets;
			
			float m_time_last_update;
			
		public:
			ParticleFilter(const std::string& type = "particle", const std::string& name = "unnamed_pf") : SensorFilter(type,name), Localizer(0) {;}
			
			virtual ~ParticleFilter()
			{
				for (std::list<PTracking::PriorityBuffer<PairPriorityBuffer>*>::iterator it = historyEstimatedTargets.begin(); it != historyEstimatedTargets.end(); ++it)
				{
					delete *it;
				}
			}
			
			void checkHistoryEstimatedTargetForCleaning(unsigned long,unsigned long);
			virtual void configure(const std::string&);
			std::vector<PTracking::PolarPoint> findTrajectoriesEstimatedTargets(PTracking::Point2of,unsigned long,unsigned long,float);
			std::vector<std::pair<PoseParticleVector,PTracking::Point2of> >& getClusters() { return clusters; }
			SensorModel* getSensorModel() { return m_sensorModel; }
			float getTimeOfLastUpdate() { return m_time_last_update; }
			virtual void initFromReadings(const std::vector<GMapping::SensorReading*>&);
			virtual void initFromUniform();
			
			virtual void observe(const GMapping::SensorReading* reading)
			{
				resetWeight();
				
				if (Localizer::observe(*m_sensorModel,reading)) resample();
			}
			
			virtual void predict(const PTracking::PointWithVelocity&, const PTracking::PointWithVelocity&) {;}
			
			void setClusters(std::vector<std::pair<PoseParticleVector,PTracking::Point2of> >&);
			void setParticles(PoseParticleVector);
			void updateHistoryEstimatedTargets(unsigned long,float);
			
			FILTER_DEFAULT_CLONE(ParticleFilter)
	};
}
