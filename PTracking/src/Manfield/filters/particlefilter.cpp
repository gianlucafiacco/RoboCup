#include <Manfield/filters/particlefilter.h>
#include <Manfield/configfile/configfile.h>
#include <Manfield/utils/debugutils.h>
#include <Utils/Utils.h>
#include <fstream>

using namespace std;
using GMapping::ConfigFile;
using PTracking::Point2f;
using PTracking::Point2of;
using PTracking::PolarPoint;
using PTracking::PriorityBuffer;
using PTracking::Utils;

namespace manfield
{
	void ParticleFilter::checkHistoryEstimatedTargetForCleaning(unsigned long currentTimestamp, unsigned long validityTime)
	{
		bool clean;
		
		for (list<PriorityBuffer<PairPriorityBuffer>*>::iterator it = historyEstimatedTargets.begin(); it != historyEstimatedTargets.end(); )
		{
			clean = true;
			
			for (PriorityBuffer<PairPriorityBuffer>::const_iterator it2 = (*it)->begin(); it2 != (*it)->end(); it2++)
			{
				// See the file .h to better understand the meaning of weight.
				if ((currentTimestamp - (it2->second).weight) <= validityTime)
				{
					clean = false;
					
					break;
				}
			}
			
			if (clean) it = historyEstimatedTargets.erase(it);
			else ++it;
		}
	}
	
	void ParticleFilter::configure(const string& filename)
	{
		ConfigFile fCfg;
		string filterName = "", key, section;
		
		if (!fCfg.read(filename))
		{
			ERR("Error reading file '" << filename << "' for filter configuration. Exiting..." << endl);
			
			exit(-1);
		}
		
		section = "parameters";
		
		try
		{
			key = "filtername";
			filterName = string(fCfg.value(section, key));
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		setname(filterName);
		
		try
		{
			key = "nparticles";
			int nparticles = fCfg.value(section,key);
			
			Localizer::setparticleNumber(nparticles);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		float st0 = 0, sr0 = 0;
		
		try
		{
			key = "st0";
			st0 = fCfg.value(section,key);
			
			key = "sr0";
			sr0 = fCfg.value(section,key);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		Localizer::setMotionParameters(sr0,st0,0.0,0.0,0.0,0.0);
		
		key = "posetype";
		
		try
		{
			m_params.setposeType(LocalizerParameters::string2PoseType(fCfg.value(section,key)));
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		Point2f localizedsigma;
		
		try
		{
			key = "localizedsigmaX";
			localizedsigma.x = fCfg.value(section,key);
			
			key = "localizedsigmaY";
			localizedsigma.y = fCfg.value(section,key);
			
			m_params.setlocalizedSigma(localizedsigma);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
	}
	
	vector<PolarPoint> ParticleFilter::findTrajectoriesEstimatedTargets(Point2of target, unsigned long currentTimestamp, unsigned long validityTime, float distance)
	{
		vector<PolarPoint> trajectories;
		float deltaRho, deltaTheta, dt, dtheta, dx, dy;
		int number;
		
		for (list<PriorityBuffer<PairPriorityBuffer>*>::iterator it = historyEstimatedTargets.begin(); it != historyEstimatedTargets.end(); it++)
		{
			Point2of mean;
			
			number = 0;
			
			for (PriorityBuffer<PairPriorityBuffer>::const_iterator it2 = (*it)->begin(); it2 != (*it)->end(); it2++)
			{
				// See the file .h to better understand the meaning of weight.
				if ((currentTimestamp - (it2->second).weight) <= validityTime)
				{
					mean.x += (it2->second).estimatedTarget.x;
					mean.y += (it2->second).estimatedTarget.y;
					
					number++;
				}
			}
			
			if (number > 0)
			{
				mean.x /= number;
				mean.y /= number;
				
				if (Utils::isTargetNear(target,mean,distance))
				{
					if ((*it)->size() > 1) 
					{
						PriorityBuffer<PairPriorityBuffer>::const_iterator maxIterator = (--(*it)->end());
						PairPriorityBuffer max;
						
						(*it)->max(max);
						
						deltaRho = 0.0;
						deltaTheta = 0.0;
						number = 0;
						
						for (PriorityBuffer<PairPriorityBuffer>::const_iterator it2 = (*it)->begin(); it2 != (*it)->end(); it2++)
						{
							PriorityBuffer<PairPriorityBuffer>::const_iterator last = (--(*it)->end());
							PriorityBuffer<PairPriorityBuffer>::const_iterator secondLast = (--(--(*it)->end()));
							
							if (it2 != maxIterator)
							{
								// See the file .h to better understand the meaning of weight.
								if ((currentTimestamp - it2->second.weight) <= validityTime)
								{
									if ((last->second.estimatedTarget.mod() - secondLast->second.estimatedTarget.mod()) > 0.05)
									{
										dx = it2->second.estimatedTarget.x - max.estimatedTarget.x;
										dy = it2->second.estimatedTarget.y - max.estimatedTarget.y;
										dtheta = Utils::angNormPiSig(atan2(dy,dx));
										
										// Because the timestamp are in ms.
										dt = (static_cast<float>(max.weight - it2->second.weight) / 1000.0);
										
										if (dt > 0.0)
										{
											deltaRho += (dx / dt);
											deltaTheta += (dtheta / dt);
											deltaTheta = Utils::angNormPiSig(deltaTheta);
										}
										
										number++;
									}
									else
									{
										trajectories.push_back(PolarPoint(0.0,0.0));
									}
								}
							}
						}
						
						// This means that there is almost one estimated target not too old.
						if (number > 0) trajectories.push_back(PolarPoint(deltaTheta / number,2 * (deltaRho / number)));
						else trajectories.push_back(PolarPoint(0.0,0.0));
					}
					else
					{
						// In this case there is only one estimated pose and there is no possibility to calculate the direction.
						trajectories.push_back(PolarPoint(0.0,0.0));
					}
				}
			}
		}
		
		return trajectories;
	}
	
	void ParticleFilter::initFromReadings(const vector<GMapping::SensorReading*>&)
	{
		cerr << "Error: ParticleFilter::initFromReadings(const vector<GMapping::SensorReading*>&) not implemented yet." << endl;
	}
	
	void ParticleFilter::initFromUniform()
	{
		if (m_sensorModel == 0)
		{
			cerr << "SensorModel is not initiliazed for filter '" << gettype().c_str() << "'." << endl;
			
			return;
		}
		
		initFromMap(*m_sensorModel);
	}
	
	void ParticleFilter::setClusters(vector<pair<PoseParticleVector,Point2of> >& c)
	{
		clusters = c;
	}
	
	void ParticleFilter::setParticles(PoseParticleVector p)
	{
		m_params.m_particles = p;
		m_params.m_particleNumber = p.size();
	}
	
	void ParticleFilter::updateHistoryEstimatedTargets(unsigned long currentTimestamp, float distance)
	{
		unsigned int size;
		bool inserted;
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); it++)
		{
			inserted = false;
			
			for (list<PriorityBuffer<PairPriorityBuffer>*>::iterator it2 = historyEstimatedTargets.begin(); it2 != historyEstimatedTargets.end(); it2++)
			{
				Point2of mean;
				
				size = 0;
				
				for (PriorityBuffer<PairPriorityBuffer>::const_iterator it3 = (*it2)->begin(); it3 != (*it2)->end(); it3++)
				{
					mean.x += (it3->second).estimatedTarget.x;
					mean.y += (it3->second).estimatedTarget.y;
					
					size++;
				}
				
				if (size > 0)
				{
					mean.x /= size;
					mean.y /= size;
					
					if (Utils::isTargetNear((*it).second,mean,distance))
					{
						PairPriorityBuffer pairBuffer;
						
						pairBuffer.estimatedTarget = (*it).second;
						pairBuffer.weight = currentTimestamp;
						
						(*it2)->push(pairBuffer);
						inserted = true;
						
						break;
					}
				}
			}
			
			if (!inserted)
			{
				PriorityBuffer<PairPriorityBuffer>* buffer;
				PairPriorityBuffer pairBuffer;
				
				buffer = new PriorityBuffer<PairPriorityBuffer>(MAX_ELEMENTS_PRIORITY_BUFFER);
				
				pairBuffer.estimatedTarget = (*it).second;
				pairBuffer.weight = currentTimestamp;
				
				buffer->push(pairBuffer);
				
				historyEstimatedTargets.push_back(buffer);
			}
		}
	}
	
	SENSORFILTER_FACTORY(ParticleFilter)
}
