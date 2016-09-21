#pragma once

#include <Core/Filters/ObjectSensorReading.h>
#include <Utils/Utils.h>
#include <Manfield/filters/gmlocalizer/structs.h>
#include <inttypes.h>

namespace PTracking
{
	/**
	 * @class AgentPacket
	 * 
	 * @brief Class that defines the packet containing the information exchanged between the agents.
	 */
	class AgentPacket
	{
		public:
			/**
			 * @struct Data
			 * 
			 * @brief Struct representing a message exchanged by the team of agents. 
			 */
			struct Data
			{
				/**
				 * @brief sender agent address.
				 */
				std::string ip;
				
				/**
				 * @brief sender agent port.
				 */
				int port;
				
				/**
				 * @brief position of the sender agent.
				 */
				Point2of agentPose;
				
				/**
				 * @brief map of estimations performed by the sender agent (local estimation layer).
				 */
				std::map<int,std::pair<ObjectSensorReading::Observation,Point2f> > estimatedTargetModels;
				
				/**
				 * @brief timestamp of the particles.
				 */
				uint64_t particlesTimestamp;
			};
			
			/**
			 * @brief message containing the information of an agent to send over the network.
			 */
			Data dataPacket;
			
			/**
			 * @brief Empty constructor.
			 */
			AgentPacket() {;}
			
			/**
			 * @brief Destructor.
			 */
			~AgentPacket() {;}
			
			/**
			 * @brief Function that fills the packet fields by parsing the information contained in the string given in input.
			 * 
			 * @param s reference to the information to be sent by an agent (string format).
			 */
			inline void setData(const std::string& s)
			{
				std::stringstream app;
				int size;
				
				app << s;
				
				app >> dataPacket.ip >> dataPacket.port >> dataPacket.agentPose.x >> dataPacket.agentPose.y >> dataPacket.agentPose.theta >> size;
				
				for (int i = 0; i < size; ++i)
				{
					ObjectSensorReading::Observation o;
					int targetIdentity;
					
					app >> targetIdentity >> o.observation.rho >> o.observation.theta >> o.sigma.x >> o.sigma.y >> o.model.width >> o.model.height >> o.model.barycenter;
					
					dataPacket.estimatedTargetModels.insert(std::make_pair(targetIdentity,std::make_pair(o,o.sigma)));
				}
				
				app >> dataPacket.particlesTimestamp;
			}
			
			/**
			 * @brief Function that converts the packet information into a string.
			 * 
			 * @return a string representing the packet information.
			 */
			inline std::string toString()
			{
				std::stringstream app;
				
				app << dataPacket.ip << " " << dataPacket.port << " " << Utils::roundN(dataPacket.agentPose.x,2) << " "
					<< Utils::roundN(dataPacket.agentPose.y,2) << " " << Utils::roundN(dataPacket.agentPose.theta,2) << " "
					<< dataPacket.estimatedTargetModels.size();
				
				for (std::map<int,std::pair<ObjectSensorReading::Observation,Point2f> >::iterator it = dataPacket.estimatedTargetModels.begin(); it != dataPacket.estimatedTargetModels.end(); ++it)
				{
					app << " " << it->first << " " << Utils::roundN(it->second.first.observation.rho,2) << " " << Utils::roundN(it->second.first.observation.theta,2)
						<< " " << Utils::roundN(it->second.first.sigma.x,2) << " " << Utils::roundN(it->second.first.sigma.y,2)
						<< " " << it->second.first.model.width << " " << it->second.first.model.height << " " << it->second.first.model.barycenter;
				}
				
				app << dataPacket.particlesTimestamp;
				
				return app.str();
			}
	};
}
