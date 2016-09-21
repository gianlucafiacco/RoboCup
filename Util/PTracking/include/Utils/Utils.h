#pragma once

#include "Point2f.h"
#include "Point2of.h"
#include "PolarPoint.h"
#include <Manfield/filters/gmlocalizer/structs.h>
#include <Manfield/utils/debugutils.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <math.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <string>

namespace PTracking
{
	/**
	 * @class Utils
	 * 
	 * @brief Class that defines several useful functions.
	 */
	class Utils
	{
		public:
			/**
			 * @brief Enumerator representing all the possible decreasing model factors.
			 */
			enum DecreaseModelFactor
			{
				Exponential = 0,
				Linear,
				Logaritmic,
				Quadratic
			};
			
			/**
			 * @brief Enumerator representing all the possible filter types.
			 */
			enum FilterType
			{
				MultiObjectParticleFilter = 0,
				ObjectParticleFilter
			};
			
			/**
			 * @brief Enumerator representing all the possible object types.
			 */
			enum ObjectType
			{
				Circle = 0,
				Landmark,
				Line2dOnMap,
				Point2fOnMap,
				Point2ofOnMap
			};
			
			/**
			 * @brief Function that normalizes an angle within [-M_PI,M_PI].
			 * 
			 * @param angle angle to be normalized.
			 * 
			 * @return the normalized angle.
			 */
			inline static float angNormPiSig(float angle)
			{
				float r = std::fmod((double) angle,(double) 2 * M_PI);
				
				if (r > M_PI) r = -(2 * M_PI - r);
				else if (r < -M_PI) r = 2 * M_PI + r;
				
				return r;
			}
			
			/**
			 * @brief Function that computes the mean of each element of a vector of vectors of particles.
			 * 
			 * @param part reference to the vector of vectors of particles.
			 * 
			 * @return a vector representing the means of each element of the vector of vectors of particles given in input.
			 */
			inline static std::vector<Point2f> calculateAllMeanParticles(const std::vector<PoseParticleVector>& part)
			{
				std::vector<Point2f> allMean;
				
				for (std::vector<PoseParticleVector>::const_iterator it = part.begin(); it != part.end(); it++)
				{
					allMean.push_back(calculateMeanParticles(*it));
				}
				
				return allMean;
			}
			
			/**
			 * @brief Function that computes the standard deviation of each element of a vector of vectors of particles.
			 * 
			 * @param part reference to the vector of vectors of particles.
			 * @param allMean reference to the vector representing the means of each element of a vector of vectors of particles.
			 * 
			 * @return a vector representing the standard deviation of each element of the vector of vectors of particles given in input.
			 */
			inline static std::vector<Point2f> calculateAllSigmaParticles(const std::vector<PoseParticleVector>& part, const std::vector<Point2f>& allMean)
			{
				std::vector<Point2f> allSigma;
				int i = 0;
				
				for (std::vector<PoseParticleVector>::const_iterator it = part.begin(); it != part.end(); it++, ++i)
				{
					allSigma.push_back(calculateSigmaParticles((*it),allMean.at(i)));
				}
				
				return allSigma;
			}
			
			/**
			 * @brief Function that computes the centroid of a vector of particles.
			 * 
			 * @param particles reference to the vector of particles.
			 * 
			 * @return a point representing the centroid of the particles given in input.
			 */
			inline static Point2of calculateCentroid(const PoseParticleVector& particles)
			{
				Point2of centroid;
				int i, size;
				
				i = 0;
				size = particles.size();
				
				/// Partial Loop Unrolling to better use pipeling.
				for (; i < size - 3; i += 4)
				{
					centroid.x += particles.at(i).pose.pose.x + particles.at(i + 1).pose.pose.x +
								  particles.at(i + 2).pose.pose.x + particles.at(i + 3).pose.pose.x;
					
					centroid.y += particles.at(i).pose.pose.y + particles.at(i + 1).pose.pose.y +
								  particles.at(i + 2).pose.pose.y + particles.at(i + 3).pose.pose.y;
				}
				
				for (; i < size; ++i)
				{
					centroid.x += particles.at(i).pose.pose.x;
					centroid.y += particles.at(i).pose.pose.y;
				}
				
				if (particles.size() > 0)
				{
					centroid.x /= particles.size();
					centroid.y /= particles.size();
				}
				
				return centroid;
			}
			
			/**
			 * @brief Function that computes the mean of a vector of particles.
			 * 
			 * @param part reference to the vector of particles.
			 * 
			 * @return a point representing the mean of the vector of particles given in input.
			 */
			inline static Point2f calculateMeanParticles(const PoseParticleVector& part)
			{
				Point2f mean;
				int i, size;
				
				mean.x = 0.0;
				mean.y = 0.0;
				
				i = 0;
				size = part.size();
				
				/// Partial Loop Unrolling to better use pipeling.
				for (; i < size - 3; i += 4)
				{
					mean.x += part.at(i).pose.pose.x + part.at(i + 1).pose.pose.x + part.at(i + 2).pose.pose.x + part.at(i + 3).pose.pose.x;
					mean.y += part.at(i).pose.pose.y + part.at(i + 1).pose.pose.y + part.at(i + 2).pose.pose.y + part.at(i + 3).pose.pose.y;
				}
				
				for (; i < size; ++i)
				{
					mean.x += part.at(i).pose.pose.x;
					mean.y += part.at(i).pose.pose.y;
				}
				
				if (part.size() > 0)
				{
					mean.x /= part.size();
					mean.y /= part.size();
				}
				
				return mean;
			}
			
			/**
			 * @brief Function that computes the standard deviation of a vector of particles.
			 * 
			 * @param part reference to the vector of particles.
			 * @param mean reference to the mean of the vector of particles.
			 * 
			 * @return a point representing the standard deviation of the vector of particles given in input.
			 */
			inline static Point2f calculateSigmaParticles(const PoseParticleVector& part, const Point2f& mean)
			{
				Point2f sigma;
				int i, size;
				
				sigma.x = 0.0;
				sigma.y = 0.0;
				
				i = 0;
				size = part.size();
				
				/// Partial Loop Unrolling to better use pipeling.
				for (; i < size - 3; i += 4)
				{
					sigma.x += ((part.at(i).pose.pose.x - mean.x) * (part.at(i).pose.pose.x - mean.x)) +
							   ((part.at(i + 1).pose.pose.x - mean.x) * (part.at(i + 1).pose.pose.x - mean.x)) +
							   ((part.at(i + 2).pose.pose.x - mean.x) * (part.at(i + 2).pose.pose.x - mean.x)) +
							   ((part.at(i + 3).pose.pose.x - mean.x) * (part.at(i + 3).pose.pose.x - mean.x));
					
					sigma.y += ((part.at(i).pose.pose.y - mean.y) * (part.at(i).pose.pose.y - mean.y)) +
							   ((part.at(i + 1).pose.pose.y - mean.y) * (part.at(i + 1).pose.pose.y - mean.y)) +
							   ((part.at(i + 2).pose.pose.y - mean.y) * (part.at(i + 2).pose.pose.y - mean.y)) +
							   ((part.at(i + 3).pose.pose.y - mean.y) * (part.at(i + 3).pose.pose.y - mean.y));
				}
				
				for (; i < size; ++i)
				{
					sigma.x += ((part.at(i).pose.pose.x - mean.x) * (part.at(i).pose.pose.x - mean.x));
					sigma.y += ((part.at(i).pose.pose.y - mean.y) * (part.at(i).pose.pose.y - mean.y));
				}
				
				if (size > 0)
				{
					sigma.x = std::sqrt(sigma.x / ((float) size));
					sigma.y = std::sqrt(sigma.y / ((float) size));
				}
				
				// It's too small.
				if ((std::fabs(sigma.x) < 1e-2) || (std::fabs(sigma.y) < 1e-2))
				{
					sigma.x = 0.01;
					sigma.y = 0.01;
				}
				
				return sigma;
			}
			
			/**
			 * @brief Function that compares two pairs having as first element an integer value.
			 * 
			 * @param i reference to the first pair to be compared.
			 * @param j reference to the second pair to be compared.
			 * 
			 * @return \b true if the first pair is less than the second one, \b false otherwise.
			 */
			inline static bool comparePairInt(const std::pair<int,std::string>& i, const std::pair<int,std::string>& j)
			{
				return (i.first < j.first);
			}
			
			/**
			 * @brief Function that compares two PoseParticle objects.
			 * 
			 * @param i reference to the first object to be compared.
			 * @param j reference to the second object to be compared.
			 * 
			 * @return \b true if the first object is less than the second one, \b false otherwise.
			 */
			inline static bool comparePoseParticle(const PoseParticle& i, const PoseParticle& j)
			{
				return (i.weight > j.weight);
			}
			
			/**
			 * @brief Function that converts a cartesian point from a relative reference system to a global reference system.
			 * 
			 * @param point reference to the cartesian point to be converted.
			 * @param frame reference to the global reference frame.
			 * 
			 * @return the converted cartesian point.
			 */
			inline static Point2of convertRelative2Global(const Point2of& point, const Point2of& frame)
			{
				Point2of result;
				
				float rho = sqrt((point.x * point.x) + (point.y * point.y));
				
				result.x = frame.x + (rho * cos(frame.theta + atan2(point.y,point.x)));
				result.y = frame.y + (rho * sin(frame.theta + atan2(point.y,point.x)));
				
				return result;
			}
			
			/**
			 * @brief Function that converts an angle from degrees to radiants.
			 * 
			 * @param angle angle to be converted.
			 * 
			 * @return the converted angle.
			 */
			inline static float deg2rad(float angle)
			{
				return angle * M_PI / 180.0;
			}
			
			/**
			 * @brief Function that computes the next estimated position of a moving target.
			 * 
			 * @param targetPosition reference to the current position of the target.
			 * @param targetVelocity reference to the current velocity of the target.
			 * @param dt time between two target estimations.
			 * 
			 * @return an object representing the next estimated position of the target.
			 */
			inline static PointWithVelocity estimatedPosition(const PointWithVelocity& targetPosition, const PolarPoint& targetVelocity, float dt)
			{
				PointWithVelocity delta;
				
				delta.pose.x = targetPosition.pose.x + ((targetVelocity.rho * dt) * std::cos(angNormPiSig(targetVelocity.theta) * dt));
				delta.pose.y = targetPosition.pose.y + ((targetVelocity.rho * dt) * std::sin(angNormPiSig(targetVelocity.theta) * dt));
				
				return delta;
			}
			
			/**
			 * @brief Function that returns the enum of a filter type given in string form.
			 * 
			 * @param f filter type.
			 * 
			 * @return the corresponding enum of the filter type.
			 */
			inline static FilterType filterType(const std::string& f)
			{
				if (strcasecmp(f.c_str(),"ObjectParticleFilter") == 0) return ObjectParticleFilter;
				else if (strcasecmp(f.c_str(),"MultiObjectParticleFilter") == 0) return MultiObjectParticleFilter;
				else return ObjectParticleFilter;
			}
			
			/**
			 * @brief Function that reads a file to get all the agents' name.
			 * 
			 * The file must follow the following schema:
			 * 
			 * [Agent]
			 * Agent1Address X.X.X.X
			 * Agent1Port <port-number-1>
			 * Agent2Address Y.Y.Y.Y
			 * Agent2Port <port-number-2>
			 * ...
			 * AgentNAddress Z.Z.Z.Z
			 * AgentNPort <port-number-n>
			 * 
			 * [Foo]
			 * Foo1Address X.X.X.X
			 * Foo1Port <port-number-1>
			 * Foo2Address Y.Y.Y.Y
			 * Foo2Port <port-number-2>
			 * ...
			 * FooMAddress Z.Z.Z.Z
			 * FooMPort <port-number-m>
			 * 
			 * ...
			 * 
			 * @param filename file to be read.
			 * @param section section of the file to be read.
			 * 
			 * @return a vector containing all the agents' name.
			 */
			inline static std::vector<std::string> getAgentsName(const std::string& filename, const std::string& section)
			{
				std::vector<std::string> agentNames;
				std::ifstream ifs;
				std::string app;
				bool sectionFound;
				
				ifs.open(filename.c_str());
				
				if (ifs.fail())		//TODO Vincenzo - Check if it works
				{
					ERR("Error: failed to open input file " << filename << std::endl);
				}
				else
				{
					sectionFound = false;
					
					while (ifs.good())
					{
						if (ifs.eof()) break;
						
						ifs >> app;
						
						if (app == section) sectionFound = true;
						
						if (sectionFound)
						{
							ifs >> app;
							
							unsigned long ret = app.find("Address");
							
							if (ret == std::string::npos)
							{
								if (app.find("Port") == std::string::npos) break;
							}
							else agentNames.push_back(app.substr(0,ret));
						}
					}
					
					ifs.close();
				}
				
				return agentNames;
			}
			
			/**
			 * @brief Function that checks if two targets are close each other within a maximum threshold distance.
			 * 
			 * @param p1 reference to the first point to be checked.
			 * @param p2 reference to the second point to be checked.
			 * @param distance maximum threshold distance.
			 * 
			 * @return \b true if the targets are close each other, \b false otherwise.
			 */
			inline static bool isTargetNear(const Point2f& p1, const Point2f& p2, float distance)
			{
				return (std::sqrt(((p1.x - p2.x) * (p1.x - p2.x)) + ((p1.y - p2.y) * (p1.y - p2.y))) < distance);
			}
			
			/**
			 * @brief Function that converts an angle from radiants to degrees.
			 * 
			 * @param angle angle to be converted.
			 * 
			 * @return the converted angle.
			 */
			inline static float rad2deg(float angle)
			{
				return angle * 180.0 / M_PI;
			}
			
			/**
			 * @brief Function that generates a random number within [-r/2,r/2) centered in 0.
			 * 
			 * @param r range of the interval.
			 * 
			 * @return the generated random nunber.
			 */
			inline static float randr(float r)
			{
				return (((float) rand() / RAND_MAX) - 0.5) * r;
			}
			
			/**
			 * @brief Function that generates a random number within [-r/2,r/2) centered in c.
			 * 
			 * @param r range of the interval.
			 * @param c center of the interval.
			 * 
			 * @return the generated random nunber.
			 */
			inline static float randrc(float r, float c)
			{
				return (((float) rand() / RAND_MAX) - 0.5) * r + c;
			}
			
			/**
			 * @brief Function that truncates a floating number to a specified decimal position.
			 * 
			 * @param d floating number to be truncated.
			 * @param n desired decimal position.
			 * 
			 * @return the truncated floating number.
			 */
			inline static float roundN(float d, int n)
			{
				if (n == 0) return round(d);
				else if (n > 0)
				{
					float p;
					int temp;
					
					p = std::pow(10.0,n);
					temp = (int) (d * p);
					
					return (((float) temp) / p);
				}
				else return d;
			}
			
			/**
			 * @brief Function that generates a Gaussian number with a Gaussian distribution having zero mean and a specified standard deviation.
			 * 
			 * @param sigma standard deviation of the Gaussian distribution.
			 * 
			 * @return the generated Gaussian number.
			 */
			inline static float sampleGaussianSigma(float sigma)
			{
				static gsl_rng* r = 0;
		
				if (r == 0)
				{
					gsl_rng_env_setup();
					r = gsl_rng_alloc (gsl_rng_default); 
				}
		
				return gsl_ran_gaussian(r,sigma);
			}
			
			/**
			 * @brief Function that samples a vector of vectors of particles having a vector of means, a vector of standard deviations and the number of particles for each vector.
			 * 
			 * @param mean reference to the vector of means.
			 * @param sigma reference to the vector of standard deviations.
			 * @param n particles number for each vector.
			 * 
			 * @return a vector of vectors of particles sampled using the values given in input.
			 */
			inline static std::vector<PoseParticleVector> samplingParticles(const std::vector<Point2f>& mean, const std::vector<Point2f>& sigma, int n)
			{
				std::vector<PoseParticleVector> particles;
				
				for (unsigned int i = 0; i < mean.size(); ++i)
				{
					particles.push_back(samplingParticles(mean.at(i),sigma.at(i),n));
				}
				
				return particles;
			}
			
			/**
			 * @brief Function that samples a vector of particles having a mean, a standard deviation and the number of particles of the vector.
			 * 
			 * @param mean reference to the mean of the vector.
			 * @param sigma reference to the standard deviation of the vector.
			 * @param n particles number of the vector.
			 * 
			 * @return a vector of particles sampled using the values given in input.
			 */
			inline static PoseParticleVector samplingParticles(const Point2f& mean, const Point2f& sigma, int n)
			{
				PoseParticleVector particles;
				
				for (int i = 0; i < n; i++)
				{
					PointWithVelocity poseParticle;
					
					poseParticle.pose.x = mean.x + Utils::sampleGaussianSigma(sigma.x);
					poseParticle.pose.y = mean.y + Utils::sampleGaussianSigma(sigma.y);
					poseParticle.pose.theta = 0.0;
					
					particles.push_back(PoseParticle(poseParticle,1.0));
				}
				
				return particles;
			}
	};
}
