#include "PTracker.h"
#include <Core/Sensors/BasicSensor.h>
#include <Utils/UdpSocket.h>
#include <Manfield/configfile/configfile.h>
#include <sys/stat.h>
#include <signal.h>
#include <string.h>
#include <algorithm>
#include <ctime>
#include <iomanip>

// Uncomment to enable debug prints.
//#define DEBUG_MODE ;

using namespace std;
using namespace PTracking;
using GMapping::ConfigFile;

PTracker::PTracker() : agentId(-1)
{
	signal(SIGINT,PTracker::interruptCallback);
	
	init();
	
	pthread_t waitAgentMessagesThreadId;
	
	pthread_create(&waitAgentMessagesThreadId,0,(void*(*)(void*)) waitAgentMessagesThread,this);
}

PTracker::PTracker(int agentId) : agentId(agentId)
{
	signal(SIGINT,PTracker::interruptCallback);
	
	init();
	
	pthread_t waitAgentMessagesThreadId;
	
	pthread_create(&waitAgentMessagesThreadId,0,(void*(*)(void*)) waitAgentMessagesThread,this);
}

PTracker::~PTracker() {;}

string PTracker::buildHeader() const
{
	stringstream header;
	int differentTypes;
	
	differentTypes = estimatedTargetModelsMultiAgent.size();
	
	if (abs(currentTargetIndex - lastCurrentTargetIndex) > 0) ++differentTypes;
	
	differentTypes += objectParticleFilter.getObservationsMapping().size();
	
	header << agentId << " " << differentTypes << " ";
	
	/// Pay attention: "true" or "false" means respectively whether the point is oriented or not. In the first case you must use Point2ofOnMap.
	//header << "0" << " " << differentType << " AgentPose true #0000ff " << 7 << " " << 2.0 << " ";
	
	for (EstimationsMultiAgent::const_iterator it = estimatedTargetModelsMultiAgent.begin(); it != estimatedTargetModelsMultiAgent.end(); ++it)
	{
		stringstream s;
		
		const map<int,pair<int,pair<int,int> > >::const_iterator& colorTrack = colorMap.find(it->first);
		
		s << "#" << setw(2) << setfill('0') << std::hex << colorTrack->second.second.second
				 << setw(2) << setfill('0') << std::hex << colorTrack->second.second.first
				 << setw(2) << setfill('0') << std::hex << colorTrack->second.first;
		
		header << "EstimatedTargetModelsWithIdentityMultiAgent false " << s.str() << " " << 6 << " " << (1.5 * agentId) << " ";
	}
	
	if (abs(currentTargetIndex - lastCurrentTargetIndex) > 0)
	{
		header << "TargetPerceptions false #0000ff " << 14 << " " << 2 << " ";
	}
	
	const map<int,pair<ObjectSensorReading::Observation,Point2of> >& observationsMapping = objectParticleFilter.getObservationsMapping();
	
	for (map<int,pair<ObjectSensorReading::Observation,Point2of> >::const_iterator it = observationsMapping.begin(); it != observationsMapping.end(); ++it)
	{
		header << "ObservationsMapping false #000000 " << 0 << " " <<  2 << " ";
	}
	
	return header.str();
}

void PTracker::configure()
{
	vector<int> agentVector;
	ConfigFile fCfg;
	stringstream s;
	string agents, key, section, temp;
	bool isPresent;
	
	if (!fCfg.read(string(getenv("PTracking_ROOT")) + string("/../config/agent.cfg")))
	{
		ERR("Error reading file '" << string(getenv("PTracking_ROOT")) << string("/../config/agent.cfg") << "' for PTracker configuration. Exiting..." << endl);
		
		exit(-1);
	}
	
	try
	{
		section = "parameters";
		
		/// It could have been set by using the constructor.
		if (agentId == -1)
		{
			key = "agentId";
			agentId = fCfg.value(section,key);
		}
		
		key = "agents";
		agents = string(fCfg.value(section,key));
		
		s << agents;
		
		while (s.good())
		{
			string temp;
			
			if (s.eof()) break;
			
			getline(s,temp,',');
			
			agentVector.push_back(atoi(temp.c_str()));
		}
		
		key = "messageFrequency";
		messageFrequency = fCfg.value(section,key);
		
		section = "[Agent]";
		
		const vector<string>& agentNames = Utils::getAgentsName(string(getenv("PTracking_ROOT")) + string("/../config/agent.cfg"),section);
		
		int counter = 1;
		
		section = "Agent";
		isPresent = false;
		
		for (vector<string>::const_iterator it = agentNames.begin(); it != agentNames.end(); it++)
		{
			if (find(agentVector.begin(),agentVector.end(),counter) == agentVector.end()) continue;
			
			key = (*it) + "Address";
			const string address = fCfg.value(section,key);
			
			key = (*it) + "Port";
			int p = fCfg.value(section,key);
			
			/// Checking if the agentId is present in the receivers' list. If so, the information between the local and global layer are exchanged by using the main memory.
			if (counter++ == agentId)
			{
				isPresent = true;
				agentAddress = address;
				agentPort = p;
				
				continue;
			}
			
			WARN("Adding receiver: " << address << ":" << p << endl);
			
			receivers.push_back(make_pair(address,p));
		}
		
		if (!isPresent)
		{
			ERR("The agent id " << agentId << " is not present in the list of the receivers... Please check the configuration! Exiting..." << endl);
			
			exit(-1);
		}
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	if (!fCfg.read(string(getenv("PTracking_ROOT")) + string("/../config/parameters.cfg")))
	{
		ERR("Error reading file '" << string(getenv("PTracking_ROOT")) << string("/../config/parameters.cfg") << "' for PTracker configuration. Exiting..." << endl);
		
		exit(-1);
	}
	
	try
	{
		section = "parameters";
		
		key = "bestParticles";
		bestParticlesNumber = fCfg.value(section,key);
		
		section = "sensor";
		
		key = "maxReading";
		maxReading = fCfg.value(section,key);
		
		section = "location";
		
		key = "worldXMin";
		temp = string(fCfg.value(section,key));
		
		if (temp == "-inf") worldX.x = FLT_MIN;
		else worldX.x = atof(temp.c_str());
		
		key = "worldXMax";
		temp = string(fCfg.value(section,key));
		
		if (temp == "inf") worldX.y = FLT_MAX;
		else worldX.y = atof(temp.c_str());
		
		key = "worldYMin";
		temp = string(fCfg.value(section,key));
		
		if (temp == "-inf") worldY.x = FLT_MIN;
		else worldY.x = atof(temp.c_str());
		
		key = "worldYMax";
		temp = string(fCfg.value(section,key));
		
		if (temp == "inf") worldY.y = FLT_MAX;
		else worldY.y = atof(temp.c_str());
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	sizeMapX = worldX.y - worldX.x;
	sizeMapY = worldY.y - worldY.x;
	
	if (!fCfg.read(string(getenv("PTracking_ROOT")) + string("/../config/pviewer.cfg")))
	{
		ERR("Error reading file '" << string(getenv("PTracking_ROOT")) << string("/../config/pviewer.cfg") << "' for PTracker configuration. Exiting..." << endl);
		
		exit(-1);
	}
	
	try
	{
		section = "PViewer";
		
		key = "address";
		pViewerAddress = string(fCfg.value(section,key));
		
		key = "port";
		pViewerPort = fCfg.value(section,key);
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
}

void PTracker::init()
{
	int counter;
	
	configure();
	
	initialTimestamp.setToNow();
	initialTimestampMas.setToNow();
	currentTimestamp.setToNow();
	counterResult = 0;
	currentTargetIndex = 0;
	iterationCounter = 0;
	lastCurrentTargetIndex = 0;
	lastTargetIndex = -1;
	maxTargetIndex = 0;
	
	processor.addSensorFilter(&objectParticleFilter);
	
	objectParticleFilter.configure(string(getenv("PTracking_ROOT")) + string("/../config/parameters.cfg"));
	objectParticleFilter.initFromUniform();
	
	processor.init();
	
	multiAgentProcessor.addSensorFilter(&objectParticleFilterMultiAgent);
	
	objectParticleFilterMultiAgent.configure(string(getenv("PTracking_ROOT")) + string("/../config/parameters.cfg"));
	objectParticleFilterMultiAgent.initFromUniform();
	
	multiAgentProcessor.init();
	
	objectSensorReading.setSensor(objectParticleFilter.getSensor());
	
	srand(time(0));
	
	counter = 1;
	
	for (int i = 0; i < 256; i += 63)
	{
		for (int j = 0; j < 256; j += 63)
		{
			for (int k = 0; k < 256; k += 128, ++counter)
			{
				colorMap.insert(make_pair(counter,make_pair(i,make_pair(j,k))));
			}
		}
	}
}

void PTracker::interruptCallback(int)
{
	ERR(endl << "*********************************************************************" << endl);
	ERR("Caught Ctrl^C. Exiting..." << endl);
	ERR("*********************************************************************" << endl);
	
	exit(0);
}

void PTracker::exec(const ObjectSensorReading& visualReading)
{
	static UdpSocket senderSocket;
	
	vector<ObjectSensorReading> observations;
	string dataToSend;
	int ret;
	
#ifdef DEBUG_MODE
	INFO("[PTracker (" << agentId << ")] - ***********************************" << endl);
	INFO("[PTracker (" << agentId << ")] - \tNEW ITERATION (" << ++counterResult << ")" << endl);
	INFO("[PTracker (" << agentId << ")] - ***********************************" << endl);
#else
	INFO(".");
#endif
	
	currentTimestamp.setToNow();
	
	updateTargetVector(visualReading);
	
	agentPose.x = 0.0;
	agentPose.y = 0.0;
	agentPose.theta = 0.0;
	
	objectSensorReading.setObservationsAgentPose(agentPose);
	objectSensorReading.setObservations(targetVector,currentTargetIndex,lastCurrentTargetIndex,LAST_N_TARGET_PERCEPTIONS,worldX,worldY);
	
	observations.push_back(objectSensorReading);
	
	processor.processReading(agentPose,initialTimestamp,currentTimestamp,observations);
	
	const EstimationsSingleAgent& estimationsWithModel = objectParticleFilter.getEstimationsWithModel();
	
	updateTargetPosition(estimationsWithModel);
	bestParticles = updateBestParticles(estimationsWithModel);
	
	if (estimatedTargetModels.size() > 0)
	{
		dataToSend = "Agent ";
		
		AgentPacket agentPacket;
		
		agentPacket.dataPacket.ip = agentAddress;
		agentPacket.dataPacket.port = agentPort;
		agentPacket.dataPacket.agentPose = agentPose;
		agentPacket.dataPacket.estimatedTargetModels = estimatedTargetModels;
		agentPacket.dataPacket.particlesTimestamp = currentTimestamp.getMsFromMidnight();
		
		dataToSend += agentPacket.toString();
		
		if ((Timestamp() - lastTimeInformationSent).getMs() > (1000.0 / messageFrequency))
		{
			sendEstimationsToAgents(dataToSend);
			
			lastTimeInformationSent.setToNow();
		}
		
		ObjectSensorReadingMultiAgent objectSensorReadingMultiAgent;
		
		objectSensorReadingMultiAgent.setAgent(agentAddress,agentPort);
		objectSensorReadingMultiAgent.setSensor(objectParticleFilter.getSensor());
		objectSensorReadingMultiAgent.setEstimationsWithModels(estimatedTargetModels);
		objectSensorReadingMultiAgent.setEstimationsTimestamp(currentTimestamp.getMsFromMidnight());
		
		mutex.lock();
		
		observationsMultiAgent.push_back(objectSensorReadingMultiAgent);
		
		mutex.unlock();
	}
	
	initialTimestamp = currentTimestamp;
	++iterationCounter;
	
	if (iterationCounter == 1)
	{
		iterationCounter = 0;
		initialTimestampMas = currentTimestamp;
		
		mutex.lock();
		
		multiAgentProcessor.processReading(observationsMultiAgent);
		estimatedTargetModelsMultiAgent = objectParticleFilterMultiAgent.getEstimationsWithModel();
		
		observationsMultiAgent.clear();
		
		mutex.unlock();
		
		dataToSend = prepareDataForViewer();
		
		ret = senderSocket.send(dataToSend,InetAddress(pViewerAddress,pViewerPort));
		
		if (ret == -1)
		{
			ERR("Error when sending message to PViewer." << endl);
		}
	}
	
	lastCurrentTargetIndex = currentTargetIndex;
	
#ifdef DEBUG_MODE
	for (EstimationsMultiAgent::const_iterator it = estimatedTargetModelsMultiAgent.begin(); it != estimatedTargetModelsMultiAgent.end(); ++it)
	{
		WARN("[PTracker (" << agentId << ")] - target estimation global frame -> (" << it->first << ",[" << it->second.first.first.observation.getCartesian().x << ","
																										 << it->second.first.first.observation.getCartesian().y << "],"
																										 << "velocity = [" << it->second.first.first.model.velocity.x << ","
																										 << it->second.first.first.model.velocity.y << "])" << endl);
	}
	
	INFO("Time: " << (Timestamp() - currentTimestamp).getMs() << endl);
#endif
}

void PTracker::exec(const string& observationFile)
{
	ofstream result;
	ifstream file;
	string resultFile;
	char buffer[1024];
	
	file.open(observationFile.c_str());
	
	if (!file.is_open())
	{
		ERR("Error reading file '" << observationFile << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	struct stat temp;
	
	if (stat("../results",&temp) == -1)
	{
		mkdir("../results",0700);
	}
	
	resultFile = string("../results/Results_") + observationFile.substr(observationFile.rfind("/") + 1);
	
	result.open(resultFile.c_str());
	
	while (!file.eof())
	{
		stringstream dataStream;
		
		/// Reading a new line of the input file.
		file.getline(buffer,1024);
		
		if (file.eof()) continue;
		
#ifdef DEBUG_MODE
		INFO("[PTracker (" << agentId << ")] - ***********************************" << endl);
		INFO("[PTracker (" << agentId << ")] - \tNEW ITERATION (" << counterResult << ")" << endl);
		INFO("[PTracker (" << agentId << ")] - ***********************************" << endl);
#else
		INFO(".");
#endif
		
		int strBegin = string(buffer).find_first_not_of(' ');
		int strEnd = string(buffer).find_last_not_of(' ');
		
		dataStream << string(buffer).substr(strBegin,strEnd - strBegin - 1);
		
		const ObjectSensorReading& visualReading = updateVisualReadings(dataStream);
		
		exec(visualReading);
		
		if (result.is_open())
		{
			result << "Iteration " << counterResult++ << endl;
			
			for (EstimationsMultiAgent::const_iterator it = estimatedTargetModelsMultiAgent.begin(); it != estimatedTargetModelsMultiAgent.end(); ++it)
			{
				result << it->first << " " << it->second.first.first.observation.getCartesian().x << " " << it->second.first.first.observation.getCartesian().y << endl;
			}
		}
		
		usleep(30e3);
	}
	
	if (result.is_open())
	{
		INFO(endl << "Results saved: ");
		WARN(resultFile << endl)
	}
	else ERR(endl << "An error occured during the writing process... Results are not available." << endl);
}

string PTracker::prepareDataForViewer() const
{
	stringstream streamDataToSend;
	int i;
	
	streamDataToSend << buildHeader();
	
	//streamDataToSend << "1 " << Utils::Point2ofOnMap << " " << Utils::roundN(agentPose.x,2) << " " << Utils::roundN(agentPose.y,2) << " " << Utils::roundN(agentPose.theta,2);
	
	for (EstimationsMultiAgent::const_iterator it = estimatedTargetModelsMultiAgent.begin(); it != estimatedTargetModelsMultiAgent.end(); ++it)
	{
		streamDataToSend << " 1 " << Utils::Point2fOnMap << " " << Utils::roundN(it->second.first.first.observation.getCartesian().x,2) << " " << Utils::roundN(it->second.first.first.observation.getCartesian().y,2);
	}
	
	if (abs(currentTargetIndex - lastCurrentTargetIndex) > 0)
	{
		streamDataToSend << " " << ((lastCurrentTargetIndex > currentTargetIndex) ? ((LAST_N_TARGET_PERCEPTIONS - lastCurrentTargetIndex) + currentTargetIndex)
																				  : (currentTargetIndex - lastCurrentTargetIndex));
	}
	
	i = lastCurrentTargetIndex;
	
	while (i != currentTargetIndex)
	{
		streamDataToSend << " " << Utils::Point2fOnMap << " " << Utils::roundN(targetVector[i].observation.getCartesian().x,2) << " " << Utils::roundN(targetVector[i].observation.getCartesian().y,2);
		
		++i;
		
		if (i == LAST_N_TARGET_PERCEPTIONS) i = 0;
	}
	
	const map<int,pair<ObjectSensorReading::Observation,Point2of> >& observationsMapping = objectParticleFilter.getObservationsMapping();
	
	for (map<int,pair<ObjectSensorReading::Observation,Point2of> >::const_iterator it = observationsMapping.begin(); it != observationsMapping.end(); ++it)
	{
		streamDataToSend << " 1 " << Utils::Line2dOnMap << " " << Utils::roundN(it->second.first.observation.getCartesian().x,2) << " " << Utils::roundN(it->second.first.observation.getCartesian().y,2)
						 << " " << Utils::roundN(it->second.second.x,2) << " " << Utils::roundN(it->second.second.y,2);
	}
	
	return streamDataToSend.str();
}

void PTracker::sendEstimationsToAgents(const string& dataToSend) const
{
	UdpSocket senderSocket;
	int ret;
	
	for (vector<pair<string,int> >::const_iterator it = receivers.begin(); it != receivers.end(); it++)
	{
		ret = senderSocket.send(dataToSend,InetAddress(it->first,it->second));
		
		if (ret == -1)
		{
			ERR("PTreacker: Error when sending message to: '" << it->second << "'." << endl);
		}
	}
}

vector<PoseParticleVector> PTracker::updateBestParticles(const EstimationsSingleAgent& estimationsWithModel)
{
	bestParticles.clear();
	
	for (EstimationsSingleAgent::const_iterator it = estimationsWithModel.begin(); it != estimationsWithModel.end(); ++it)
	{
		bestParticles.push_back(Utils::samplingParticles(it->second.first.observation.getCartesian(),it->second.first.sigma,bestParticlesNumber));
	}
	
	return bestParticles;
}

void PTracker::updateTargetPosition(const EstimationsSingleAgent& estimationsWithModel)
{
	estimatedTargetModels.clear();
	
	for (EstimationsSingleAgent::const_iterator it = estimationsWithModel.begin(); it != estimationsWithModel.end(); it++)
	{
		estimatedTargetModels.insert(*it);
		
#ifdef DEBUG_MODE
		ERR("[PTracker (" << agentId << ")] - target estimation local frame -> (" << it->first << ",[" << it->second.first.observation.getCartesian().x << ","
																									   << it->second.first.observation.getCartesian().y << "])" << endl);
#endif
	}
}

void PTracker::updateTargetVector(const ObjectSensorReading& visualReading)
{
	const vector<ObjectSensorReading::Observation>& observations = visualReading.getObservations();
	
	for (vector<ObjectSensorReading::Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
	{
#ifdef DEBUG_MODE
		DEBUG("[PTracker (" << agentId << ")] - observation -> (" << it->observation.getCartesian().x << "," << it->observation.getCartesian().y << ")" << endl);
#endif
		
		targetVector[currentTargetIndex] = *it;
		
		currentTargetIndex++;
		lastTargetIndex++;
		
		if (currentTargetIndex == LAST_N_TARGET_PERCEPTIONS) currentTargetIndex = 0;
		
		if (lastTargetIndex == LAST_N_TARGET_PERCEPTIONS) lastTargetIndex = 0;
		
		if (maxTargetIndex < LAST_N_TARGET_PERCEPTIONS) maxTargetIndex++;
	}
}

ObjectSensorReading PTracker::updateVisualReadings(stringstream& dataStream)
{
	vector<ObjectSensorReading::Observation> observations;
	ObjectSensorReading visualReading;
	float x, y;
	
	while (dataStream.good())
	{
		if (dataStream.eof()) break;
		
		ObjectSensorReading::Observation obs;
		
		dataStream >> x >> y;
		
		obs.observation.rho = sqrt((x * x) + (y * y));
		obs.observation.theta = atan2(y,x);
		
		observations.push_back(obs);
	}
	
	visualReading.setObservations(observations);
	
	return visualReading;
}

void PTracker::waitAgentMessages()
{
	ObjectSensorReadingMultiAgent objectSensorReadingMultiAgent;
	UdpSocket receiverSocket;
	InetAddress sender;
	string dataReceived;
	int ret;
	bool binding;
	
	binding = receiverSocket.bind(agentPort);
	
	if (!binding)
	{
		ERR("Error during the binding operation. Data Fusion among agents is not possible...exiting!" << endl);
		
		exit(-1);
	}
	
	WARN("Agent " << agentId << " bound on port: " << agentPort << endl);
	
	objectSensorReadingMultiAgent.setSensor(objectSensorReading.getSensor());
	
	while (true)
	{
		ret = receiverSocket.recv(dataReceived,sender);
		
		if (ret == -1)
		{
			ERR("Error in receiving message from: '" << sender.toString() << "'." << endl);
			
			continue;
		}
		
		AgentPacket ap;
		
		ap.setData(dataReceived.substr(dataReceived.find(" ") + 1));
		
		objectSensorReadingMultiAgent.setAgent(ap.dataPacket.ip,ap.dataPacket.port);
		objectSensorReadingMultiAgent.setEstimationsWithModels(ap.dataPacket.estimatedTargetModels);
		objectSensorReadingMultiAgent.setEstimationsTimestamp(ap.dataPacket.particlesTimestamp);
		
		mutex.lock();
		
		observationsMultiAgent.push_back(objectSensorReadingMultiAgent);
		
		mutex.unlock();
	}
}
