#pragma once

#include <Core/Processors/Processor.h>
#include <Core/Processors/MultiAgentProcessor.h>
#include <Utils/AgentPacket.h>
#include <boost/thread/mutex.hpp>

/**
 * @class PTracker
 * 
 * @brief Class that implements a distributed tracker using a distributed particle filtering method.
 * 
 * PTracker allows to perform the distributed tracking in two ways:
 *	- by giving in input a file containing all the observations
 *	- by calling an exec function that performs the distributed tracking iteration by iteration
 */
class PTracker
{
	private:
		/**
		 * @brief human-readable typedef of the estimations performed by the team of agents.
		 */
		typedef std::map<int,std::pair<std::pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f>,std::pair<std::string,int> > > EstimationsMultiAgent;
		
		/**
		 * @brief human-readable typedef of the estimations performed by the agent.
		 */
		typedef std::map<int,std::pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f> > EstimationsSingleAgent;
		
		/**
		 * @brief maximum size of the circular buffer containing the observations coming from the sensors.
		 */
		static const int LAST_N_TARGET_PERCEPTIONS = 20;
		
		/**
		 * @brief map representing the estimations having both an identity and a model of the estimations performed by the team of agents.
		 */
		EstimationsMultiAgent estimatedTargetModelsMultiAgent;
		
		/**
		 * @brief map of estimations having both an identity and a model performed by the agent.
		 */
		EstimationsSingleAgent estimatedTargetModels;
		
		/**
		 * @brief vector of the best particles.
		 */
		std::vector<PoseParticleVector> bestParticles;
		
		/**
		 * @brief vector containing all the addresses of the agents that have to receive the information.
		 */
		std::vector<std::pair<std::string,int> > receivers;
		
		/**
		 * @brief vector containing all the estimations performed the team of agents.
		 */
		std::vector<PTracking::ObjectSensorReadingMultiAgent> observationsMultiAgent;
		
		/**
		 * @brief map containing the mapping between identity and color.
		 */
		std::map<int,std::pair<int,std::pair<int,int> > > colorMap;
		
		/**
		 * @brief single agent processor.
		 */
		PTracking::Processor processor;
		
		/**
		 * @brief multi agent processor.
		 */
		PTracking::MultiAgentProcessor multiAgentProcessor;
		
		/**
		 * @brief particle filter for the local estimation layer.
		 */
		PTracking::ObjectParticleFilter objectParticleFilter;
		
		/**
		 * @brief particle filter for the global estimation layer.
		 */
		PTracking::ObjectParticleFilterMultiAgent objectParticleFilterMultiAgent;
		
		/**
		 * @brief circular buffer containing the observations coming from the sensors.
		 */
		PTracking::ObjectSensorReading::Observation targetVector[LAST_N_TARGET_PERCEPTIONS];
		
		/**
		 * @brief model of the sensor reading.
		 */
		PTracking::ObjectSensorReading objectSensorReading;
		
		/**
		 * @brief position of the agent.
		 */
		PTracking::Point2of agentPose;
		
		/**
		 * @brief maximum admissible range for the x coordinate.
		 */
		PTracking::Point2f worldX;
		
		/**
		 * @brief maximum admissible range for the y coordinate.
		 */
		PTracking::Point2f worldY;
		
		/**
		 * @brief timestamp of the current iteration.
		 */
		PTracking::Timestamp currentTimestamp;
		
		/**
		 * @brief timestamp representing the starting time of the iteration.
		 */
		PTracking::Timestamp initialTimestamp;
		
		/**
		 * @brief timestamp representing the starting time of the multi agent iteration.
		 */
		PTracking::Timestamp initialTimestampMas;
		
		/**
		 * @brief timestamp representing the time when the last information to the team of agents have been sent.
		 */
		PTracking::Timestamp lastTimeInformationSent;
		
		/**
		 * @brief semaphore to handle the mutual exclusion between the single agent and the multi agent phase.
		 */
		boost::mutex mutex;
		
		/**
		 * @brief address of the agent.
		 */
		std::string agentAddress;
		
		/**
		 * @brief address of PViewer.
		 */
		std::string pViewerAddress;
		
		/**
		 * @brief maximum range of the sensor.
		 */
		float maxReading;
		
		/**
		 * @brief frequency by which the information are sent to the team of agents.
		 */
		float messageFrequency;
		
		/**
		 * @brief maximum x coordinate of the environment.
		 */
		float sizeMapX;
		
		/**
		 * @brief maximum y coordinate of the environment.
		 */
		float sizeMapY;
		
		/**
		 * @brief id of the agent.
		 */
		int agentId;
		
		/**
		 * @brief port of the agent.
		 */
		int agentPort;
		
		/**
		 * @brief number of best particles.
		 */
		int bestParticlesNumber;
		
		/**
		 * @brief counter used when writing the file of the results.
		 */
		int counterResult;
		
		/**
		 * @brief current index of the circular buffer of the observations.
		 */
		int currentTargetIndex;
		
		/**
		 * @brief counter of the iterations.
		 */
		int iterationCounter;
		
		/**
		 * @brief last current index of the circular buffer of the observations.
		 */
		int lastCurrentTargetIndex;
		
		/**
		 * @brief last target index of the circular buffer of the observations.
		 */
		int lastTargetIndex;
		
		/**
		 * @brief maximum index of the circular buffer of the observations.
		 */
		int maxTargetIndex;
		
		/**
		 * @brief port of PViewer.
		 */
		int pViewerPort;
		
		/**
		 * @brief Function that invokes a thread-function that waits messages coming from other agents.
		 * 
		 * @param pTracker pointer to the invocation object.
		 * 
		 * @return 0 if succeeded, -1 otherwise.
		 */
		static void* waitAgentMessagesThread(PTracker* pTracker) { pTracker->waitAgentMessages(); return 0; }
		
		/**
		 * @brief Function that allows a clean exit intercepting the SIGINT signal.
		 */
		static void interruptCallback(int);
		
		/**
		 * @brief Function that constructs the header of the message to send to PViewer.
		 * 
		 * @return the header of the message for PViewer.
		 */
		std::string buildHeader() const;
		
		/**
		 * @brief Function that reads a config file in order to initialize several configuration parameters.
		 */
		void configure();
		
		/**
		 * @brief Function that initializes several configuration parameters.
		 */
		void init();
		
		/**
		 * @brief Function that constructs the message for PViewer containing all the necessary information.
		 * 
		 * @return the message for PViewer.
		 */
		std::string prepareDataForViewer() const;
		
		/**
		 * @brief Function that sends the estimations to all the agents.
		 * 
		 * @param dataToSend reference to the estimations to send (string format).
		 */
		void sendEstimationsToAgents(const std::string& dataToSend) const;
		
		/**
		 * @brief Function that returns the best particles representing the current estimations performed by the single agent.
		 * 
		 * @param estimationsWithModel reference of the current estimations performed by the single agent.
		 * 
		 * @return the vector containing the best particles of the current estimations.
		 */
		std::vector<PoseParticleVector> updateBestParticles(const EstimationsSingleAgent& estimationsWithModel);
		
		/**
		 * @brief Function that updates the estimations performed by the single agent.
		 * 
		 * @param estimationsWithModel reference of the current estimations performed by the single agent.
		 */
		void updateTargetPosition(const EstimationsSingleAgent& estimationsWithModel);
		
		/**
		 * @brief Function that updates the observations coming from the sensors.
		 * 
		 * @param visualReading reference to the current observations coming from the sensors.
		 */
		void updateTargetVector(const PTracking::ObjectSensorReading& visualReading);
		
		/**
		 * @brief Function that updates the observations coming from the sensors.
		 * 
		 * @param dataStream reference to the current observations coming from the sensors.
		 * 
		 * @return the observations updated.
		 */
		PTracking::ObjectSensorReading updateVisualReadings(std::stringstream& dataStream);
		
		/**
		 * @brief Function that collects messages coming from the other agents.
		 */
		void waitAgentMessages();
		
	public:
		/**
		 * @brief Empty constructor.
		 */
		PTracker();
		
		/**
		 * @brief Constructor that takes the agent id as initialization value.
		 * 
		 * It initializes the agent id with the one given in input.
		 * 
		 * @param agentId id of the agent.
		 */
		PTracker(int agentId);
		
		/**
		 * @brief Destructor.
		 */
		~PTracker();
		
		/**
		 * @brief Function that gets the observations and perform the distributed tracking iteration by iteration. It can be stopped by pressing Ctrl^C.
		 * 
		 * @param visualReading reference to the current observations coming from the sensors.
		 */
		void exec(const PTracking::ObjectSensorReading& visualReading);
		
		/**
		 * @brief Function that reads the observation file and perform the distributed tracking. The results are written in a file. It can be stopped by pressing Ctrl^C.
		 * 
		 * @param observationFile reference to the file containing all the observations.
		 */
		void exec(const std::string& observationFile);
		
		/**
		 * @brief Function that returns the estimations performed by the single agent and by the team of agents.
		 * 
		 * @return a pair containing both the estimations performed by the single agent and by the team of agents.
		 */
		inline std::pair<EstimationsMultiAgent,EstimationsSingleAgent> getAgentEstimations() const { return std::make_pair(estimatedTargetModelsMultiAgent,estimatedTargetModels); }
		
		/**
		 * @brief Function that returns the group estimations performed by the single agent.
		 * 
		 * @return a vector containing the group estimations performed by the single agent.
		 */
		inline std::vector<std::pair<std::string,std::pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f> > > getAgentGroupEstimations() const { return objectParticleFilter.getGroupEstimationsWithModel(); }
};
