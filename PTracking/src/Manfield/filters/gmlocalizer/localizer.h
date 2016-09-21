#pragma once

#include "structs.h"
#include <Manfield/sensormodel.h>
#include <ThirdParty/GMapping/sensor/sensor_range/rangereading.h>
#include <ThirdParty/GMapping/utils/macro_params.h>
#include <ThirdParty/GMapping/particlefilter/pf.h>
#include <cmath>
#include <list>
#include <vector>

class Localizer
{
	protected:
		LocalizerParameters m_params;
		
	public:
		Localizer(unsigned int particles = 1, unsigned int pointSkip = 1);
		virtual ~Localizer() {;}
		
		const LocalizerParameters& getparams() { return m_params; }
		inline PoseParticleVector& getparticles() { return m_params.m_particles; }
		inline uint getparticleNumber() { return m_params.m_particleNumber; }
		
		virtual bool initFromMap(const manfield::SensorModelMap& map);
		virtual bool initFromMap(const manfield::SensorModel& sm);
		virtual bool initFromReadings(const manfield::SensorModelMap& map,const std::vector<GMapping::SensorReading*>& readings);
		virtual bool initFromReadings(const manfield::SensorModel& sm,const std::vector<GMapping::SensorReading*>& readings);
		virtual void initFromSamples(const PoseParticleVector& particles);
		virtual void initUniform(float xmin,float xmax,float ymin,float ymax);
		virtual bool observe(const manfield::SensorModel& model, const GMapping::SensorReading* observation);
		virtual void resample();
		void resetWeight();
		virtual void sensorResetFromReadings(const manfield::SensorModelMap& map,const std::vector<GMapping::SensorReading*>& readings,float fraction);
		
		inline void setlikelihoodSigma(float likelihoodSigma) { m_params.m_likelihoodSigma = likelihoodSigma; }
		
		inline void setMotionCalibParameters(float xx, float xy, float xt, float yx, float yy, float yt, float tx, float ty, float tt)
		{
			m_params.xx = xx;
			m_params.xy = xy;
			m_params.xt = xt;
			m_params.yx = yx;
			m_params.yy = yy;
			m_params.yt = yt;
			m_params.tx = tx;
			m_params.ty = ty;
			m_params.tt = tt;
		}
		
		inline void setMotionParameters(float r0, float t0, float rr, float  tr, float rt, float tt)
		{
			m_params.sr0 = r0;
			m_params.st0 = t0;
			m_params.srr = rr;
			m_params.str = tr;
			m_params.srt = rt;
			m_params.stt = tt;
		}
		
		inline void setparticleNumber(uint particleNumber)
		{
			m_params.m_particleNumber = particleNumber;
			m_params.m_particles.resize(m_params.m_particleNumber);
		}
		
		inline void setpointSkip(unsigned int pointSkip) { m_params.m_pointSkip = pointSkip; }
		
		inline void setunknownValue(int unknownValue) { m_params.m_unknownValue = unknownValue; }
};
