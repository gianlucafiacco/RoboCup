#pragma once

#include <ThirdParty/GMapping/utils/macro_params.h>
#include <Utils/PointWithVelocity.h>
#include <Utils/Point2f.h>
#include <iostream>
#include <string>
#include <vector>

struct PoseParticle
{
	std::vector<float> m_array;
	PTracking::PointWithVelocity pose;
	PTracking::Point2f last_association, last_association_inertial, last_pose_inertial;
	std::string sensorName;
	float cweight, lastWeight, time_last_update, weight;
	
	PoseParticle() : m_array(3,0), sensorName("unknown"), time_last_update(0), weight(12)
	{
		cweight = lastWeight = weight;
	}
	
	PoseParticle(const PTracking::PointWithVelocity& p, float w) : m_array(3,0), time_last_update(0)
	{
		pose = p;
		weight = cweight = lastWeight = w;
		sensorName = "unknown";
	}
	
	inline operator float() const { return weight; }
	
	inline PoseParticle& operator= (const PoseParticle& particle)
	{
		weight = particle.weight;
		lastWeight = particle.lastWeight;
		cweight = particle.cweight;
		m_array = particle.m_array;
		time_last_update = particle.time_last_update;
		pose = particle.pose;
		last_association = particle.last_association;
		last_pose_inertial = particle.last_pose_inertial;
		last_association_inertial = particle.last_association_inertial;
		sensorName = particle.sensorName;
		
		return *this;
	}
	
	std::vector<float>& toArray()
	{
		m_array[0] = pose.pose.x;
		m_array[1] = pose.pose.y;
		m_array[2] = weight;
		
		return m_array;
	}
};

typedef std::vector<PoseParticle> PoseParticleVector;
typedef std::vector<PoseParticle>::iterator ParticleIterator;
typedef std::vector<PoseParticle>::iterator PointIterator;

class LocalizerParameters
{
	public:
		PoseParticleVector m_particles;
		float srr, str, srt, stt, sr0, st0;
		float xx, xy, xt, yx, yy, yt, tx, ty, tt;
		float m_likelihoodSigma;
		unsigned int m_particleNumber, m_pointSkip;
		int m_unknownValue;
		
		enum PoseType
		{
			Best,
			Mean,
			RobustMean
		};
		
		static PoseType string2PoseType(const std::string& name)
		{
			if (name == "Best" || name == "best") return Best;
			else if (name == "Mean" || name == "mean") return Mean;
			else if (name == "RobustMean" || name == "robustmean") return RobustMean;
			else
			{
				std::cerr << "Implausible value for PoseType [" << name << "], returning Best" << std::endl;
				
				return Best;
			}
		}
		
		PARAM_SET_GET(PTracking::Point2f,localizedSigma,public,public,public)
		PARAM_GET(PoseParticle,maxParticle,public,public)
		PARAM_GET(PoseParticle,bestParticle,public,public)
		PARAM_SET_GET(PoseType,poseType,public,public,public)
};
