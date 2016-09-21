#pragma once

#include <Utils/Point2f.h>
#include <string>
#include <vector>

/**
 * @class PViewer
 * 
 * @brief Class that implements a debugging tool using Gnuplot.
 * 
 * PViewer allows to draw several information like:
 *	- the position in the world of each agent
 *	- the sensor's field-of-view of each agent
 *	- the particles of each agent
 *	- the targets' estimation performed by each agent
 * 
 * PViewer waits messages, that could come from all the agents, on a UDP socket and it plots the
 * information received with a frequency set in the configuration file. 
 */
class PViewer
{
	private:
		/**
		 * @brief buffer containing the information received that have to be plotted.
		 */
		std::vector<std::pair<int,std::string> > bufferDataReceived;
		
		/**
		 * @brief maximum admissible range for the x coordinate.
		 */
		PTracking::Point2f worldX;
		
		/**
		 * @brief maximum admissible range for the y coordinate.
		 */
		PTracking::Point2f worldY;
		
		/**
		 * @brief frequency by which the information are visualized.
		 */
		float visualizationFrequency;
		
		/**
		 * @brief scaling factor of the world model in order to try to have a better debug visualization.
		 */
		float worldScalingFactor;
		
		/**
		 * @brief width of the gnuplot window.
		 */
		int gnuplotWindowWidth;
		
		/**
		 * @brief height of the gnuplot window.
		 */
		int gnuplotWindowHeight;
		
		/**
		 * @brief number of objects still to be drawn.
		 */
		int objectNumber;
		
		/**
		 * @brief UDP port where messages are received.
		 */
		int port;
		
		/**
		 * @brief Function that allows a clean exit intercepting the SIGINT signal.
		 */
		static void interruptCallback(int);
		
		/**
		 * @brief Function that reads a config file in order to initialize several configuration parameters.
		 */
		void configure();
		
		/**
		 * @brief Function that updates the data to be plotted, adding a line to be drawn represented by the points (x1,y1) and (x2,y2).
		 * 
		 * @param x1 ordinate of the first point to be drawn.
		 * @param y1 abscissa of the first point to be drawn.
		 * @param x2 ordinate of the second point to be drawn.
		 * @param y2 abscissa of the second point to be drawn.
		 * @param prepareDataForGnuPlot reference to the stream representing the data to be drawn.
		 */
		void drawLine(float x1, float y1, float x2, float y2, std::ostringstream& prepareDataForGnuPlot);
		
		/**
		 * @brief Function that updates the data to be plotted, adding a point to be drawn represented by x and y.
		 * 
		 * @param x ordinate of the point to be drawn.
		 * @param y abscissa of the point to be drawn.
		 * @param prepareDataForGnuPlot reference to the stream representing the data to be drawn.
		 */
		void drawPoint(float x, float y, std::ostringstream& prepareDataForGnuPlot);
		
		/**
		 * @brief Function that updates the data to be plotted, adding an oriented point to be drawn represented by x, y and theta.
		 * 
		 * @param x ordinate of the point to be drawn.
		 * @param y abscissa of the point to be drawn.
		 * @param theta orientation of the point to be drawn.
		 * @param prepareDataForGnuPlot reference to the stream representing the data to be drawn.
		 */
		void drawPointWithOrientation(float x, float y, float theta, std::ostringstream& prepareDataForGnuPlot);
		
	public:
		/**
		 * @brief Empty constructor.
		 * 
		 * It registrers the SIGINT callback and it configures the parameters reading a config file.
		 */
		PViewer();
		
		/**
		 * @brief Destructor.
		 */
		~PViewer();
		
		/**
		 * @brief Function that waits messages on the specified UDP port and plots the information received with a frequency of 30 Hz. It stops when either a "End" message is received or Ctrl^C is pressed.
		 */
		void exec();
};
