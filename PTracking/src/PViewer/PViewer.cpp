#include "PViewer.h"
#include <Utils/UdpSocket.h>
#include <Utils/Timestamp.h>
#include <Utils/Utils.h>
#include <Manfield/configfile/configfile.h>
#include <Manfield/utils/debugutils.h>
#include <ThirdParty/Gnuplot/GnuplotGUI.h>
#include <float.h>
#include <signal.h>

using namespace std;
using namespace PTracking;
using namespace Gnuplot;
using GMapping::ConfigFile;

PViewer::PViewer()
{
	signal(SIGINT,PViewer::interruptCallback);
	
	configure();
}

PViewer::~PViewer() {;}

void PViewer::configure()
{
	ConfigFile fCfg;
	string key, section, temp;
	
	if (!fCfg.read(string(getenv("PTracking_ROOT")) + string("/../config/parameters.cfg")))
	{
		ERR("Error reading file '" << string(getenv("PTracking_ROOT")) << string("/../config/parameters.cfg") << "' for PViewer configuration. Exiting..." << endl);
		
		exit(-1);
	}
	
	try
	{
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
	
	if (!fCfg.read(string(getenv("PTracking_ROOT")) + string("/../config/pviewer.cfg")))
	{
		ERR("Error reading file '" << string(getenv("PTracking_ROOT")) << string("/../config/pviewer.cfg") << "' for PViewer configuration. Exiting..." << endl);
		
		exit(-1);
	}
	
	try
	{
		section = "PViewer";
		
		key = "port";
		port = fCfg.value(section,key);
		
		key = "visualizationFrequency";
		visualizationFrequency = fCfg.value(section,key);
		
		key = "worldScalingFactor";
		worldScalingFactor = fCfg.value(section,key);
		
		key = "gnuplotWindowWidth";
		gnuplotWindowWidth = fCfg.value(section,key);
		
		key = "gnuplotWindowHeight";
		gnuplotWindowHeight = fCfg.value(section,key);
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	WARN("World size: [" << (worldX.y - worldX.x) << "," << (worldY.y - worldY.x) << "]" << endl);
	WARN("World scaling factor: " << worldScalingFactor << endl);
	
	worldX.x /= worldScalingFactor;
	worldX.y /= worldScalingFactor;
	worldY.x /= worldScalingFactor;
	worldY.y /= worldScalingFactor;
	
	ERR("Scaled world size: [" << (worldX.y - worldX.x) << "," << (worldY.y - worldY.x) << "]" << endl);
	
	DEBUG("Gnuplot window: " << gnuplotWindowWidth << " x " << gnuplotWindowHeight << endl);
}

void PViewer::drawLine(float x1, float y1, float x2, float y2, ostringstream& prepareDataForGnuPlot)
{
	prepareDataForGnuPlot << x1 << " " << y1 << endl << x2 << " " << y2 << endl;
	
	objectNumber--;
	
	if (objectNumber == 0) prepareDataForGnuPlot << "e" << endl;
}

void PViewer::drawPoint(float x, float y, ostringstream& prepareDataForGnuPlot)
{
	prepareDataForGnuPlot << x << " " << y << endl;
	
	objectNumber--;
	
	if (objectNumber == 0) prepareDataForGnuPlot << "e" << endl;
}

void PViewer::drawPointWithOrientation(float x, float y, float theta, ostringstream& prepareDataForGnuPlot)
{
	static ostringstream orientation, point;
	
	point << x << " " << y << endl;
	orientation << x << " " << y << " " << cos(theta) << " " << sin(theta) << endl;
	
	objectNumber--;
	
	if (objectNumber == 0)
	{
		prepareDataForGnuPlot << point.str() << "e" << endl << orientation.str() << "e" << endl;
		
		point.str("");
		orientation.str("");
		
		point.clear();
		orientation.clear();
	}
}

void PViewer::exec()
{
	GnuplotGUI gnuplotGUI;
	UdpSocket socket;
	InetAddress sender;
	Timestamp current, initial;
	ostringstream circleDataForGnuPlot, prepareDataForGnuPlot;
	stringstream streamDataReceived;
	string color, dataReceived, isOriented, type;
	float maxReading, x, y, theta, x2, y2, width;
	int objectType, ret;
	short agentId, differentType, form, i;
	bool binding, exiting, isFov, isPresent;
	
	exiting = false;
	
	binding = socket.bind(port);
	
	if (binding)
	{
		INFO("PViewer started." << endl);
		
		initial.setToNow();
		
		while (!exiting)
		{
			ret = socket.recv(dataReceived,sender);
			
			if (ret == -1)
			{
				ERR("Error when receiving message from: '" << sender.toString() << "'." << endl);
			}
			else
			{
				if (dataReceived == "End")
				{
					exiting = true;
					
					break;
				}
				
				streamDataReceived.str("");
				streamDataReceived.clear();
				
				streamDataReceived << dataReceived;
				
				streamDataReceived >> agentId;
				
				isPresent = false;
				
				for (vector<pair<int,string> >::iterator it = bufferDataReceived.begin(); it != bufferDataReceived.end(); it++)
				{
					if (it->first == agentId)
					{
						isPresent = true;
						it->second = dataReceived;
						
						break;
					}
				}
				
				if (!isPresent) bufferDataReceived.push_back(make_pair(agentId,dataReceived));
				
				current.setToNow();
				
				if ((current - initial).getMs() > (1000.0 / visualizationFrequency))
				{
					prepareDataForGnuPlot.str("");
					prepareDataForGnuPlot.clear();
					
					circleDataForGnuPlot.str("");
					circleDataForGnuPlot.clear();
					
					// Sort the agent id in increasing order.
					sort(bufferDataReceived.begin(),bufferDataReceived.end(),Utils::comparePairInt);
					
					prepareDataForGnuPlot << "set terminal wxt size " << gnuplotWindowWidth << "," << gnuplotWindowHeight << endl;
					prepareDataForGnuPlot << "set grid" << endl;
					prepareDataForGnuPlot << "set xlabel \"Scale 1:" << worldScalingFactor << "\" textcolor rgb 'red'" << endl;
					prepareDataForGnuPlot << "set xlabel font \"Times-Roman, 15\"" << endl;
					prepareDataForGnuPlot << "set xrange[" << worldX.x << ":" << worldX.y << "]" << endl;
					prepareDataForGnuPlot << "set yrange[" << worldY.y << ":" << worldY.x << "]" << endl;
					
					prepareDataForGnuPlot << "plot ";
					
					i = 0;
					isFov = false;
					
					for (vector<pair<int,string> >::iterator it = bufferDataReceived.begin(); it != bufferDataReceived.end(); ++it, ++i)
					{
						streamDataReceived.str("");
						streamDataReceived.clear();
						
						streamDataReceived << it->second;
						
						streamDataReceived >> agentId >> differentType;
						
						if ((i > 0) && isFov) prepareDataForGnuPlot << ", ";
						
						isFov = false;
						
						if (differentType > 0)
						{
							for (short i = 0; i < differentType; i++)
							{
								streamDataReceived >> type >> isOriented >> color >> form >> width;
								
								if (type != "FOV")
								{
									if (isFov)
									{
										prepareDataForGnuPlot << ", ";
										
										isFov = false;
									}
									
									if (type != "ObservationsMapping")
									{
										prepareDataForGnuPlot << "'-' notitle '" << type << "' w p pt " << form << " ps " << width << " lt rgb '" << color << "'";
										
										if (strcasecmp(isOriented.c_str(),"true") == 0) prepareDataForGnuPlot << ", '-' notitle w vec lw 2 lt rgb '#000000'";
									}
									else prepareDataForGnuPlot << "'-' notitle '" << type << "' w l lw " << width << " lt rgb '" << color << "'";
									
									if (((it + 1) != bufferDataReceived.end()) || (i < (differentType - 1))) prepareDataForGnuPlot << ", ";
								}
								else
								{
									string temp;
									
									temp = prepareDataForGnuPlot.str();
									
									prepareDataForGnuPlot.str("");
									prepareDataForGnuPlot.clear();
									
									prepareDataForGnuPlot << temp.substr(0,temp.size() - 2);
									
									isFov = true;
								}
							}
						}
					}
					
					prepareDataForGnuPlot << endl;
					
					for (vector<pair<int,string> >::iterator it = bufferDataReceived.begin(); it != bufferDataReceived.end(); ++it)
					{
						streamDataReceived.str("");
						streamDataReceived.clear();
						
						streamDataReceived << it->second;
						
						streamDataReceived >> agentId >> differentType;
						
						if (differentType > 0)
						{
							for (short i = 0; i < differentType; i++)
							{
								streamDataReceived >> type >> isOriented >> color >> form >> width;
							}
						}
						
						objectNumber = 0;
						
						while (streamDataReceived.good())
						{
							if (streamDataReceived.eof()) break;
							
							if (objectNumber == 0) streamDataReceived >> objectNumber >> objectType;
							else streamDataReceived >> objectType;
							
							if (objectType == Utils::Point2fOnMap)
							{
								streamDataReceived >> x >> y;
								
								drawPoint(x / worldScalingFactor,y / worldScalingFactor,prepareDataForGnuPlot);
							}
							else if (objectType == Utils::Point2ofOnMap)
							{
								streamDataReceived >> x >> y >> theta;
								
								drawPointWithOrientation(x / worldScalingFactor,y / worldScalingFactor,theta,prepareDataForGnuPlot);
							}
							else if (objectType == Utils::Line2dOnMap)
							{
								streamDataReceived >> x >> y >> x2 >> y2;
								
								drawLine(x / worldScalingFactor,y / worldScalingFactor,x2 / worldScalingFactor,y2 / worldScalingFactor,prepareDataForGnuPlot);
							}
							else if (objectType == Utils::Circle)
							{
								streamDataReceived >> maxReading >> x >> y >> theta;
								
								circleDataForGnuPlot << "set object circle at " << (x / worldScalingFactor) << "," << (y / worldScalingFactor) << " size " << maxReading
													 << " arc [" << (-90 + Utils::rad2deg(theta)) << ":" << (90 + Utils::rad2deg(theta)) << "] fc rgb '#000000'" << endl;
								
								objectNumber--;
							}
							else ERR("Object type not recognized: " << objectType << endl);
						}
					}
					
					gnuplotGUI.cmd(string("unset object\n") + circleDataForGnuPlot.str() + prepareDataForGnuPlot.str());
					
					initial = current;
					bufferDataReceived.clear();
				}
			}
		}
		
		socket.shutdown(SHUT_RDWR);
	}
	else
	{
		ERR("Error in binding on port " << port << "." <<endl);
	}
}

void PViewer::interruptCallback(int)
{
	ERR(endl << "*********************************************************************" << endl);
	ERR("Caught Ctrl^C. Exiting..." << endl);
	ERR("*********************************************************************" << endl);
	
	exit(0);
}
