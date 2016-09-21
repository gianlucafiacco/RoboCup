#include "BasicSensorMap.h"
#include <Utils/Utils.h>

namespace PTracking
{
	BasicSensorMap::BasicSensorMap() {;}
	
	BasicSensorMap::~BasicSensorMap() {;}
	
	Point2f BasicSensorMap::getRandomPointInWorld() const
	{
		Point2f point;
		
		point.x = Utils::randrc(2 * getworldMax().x,0.0);
		point.y = Utils::randrc(2 * getworldMax().y,0.0);
		
		return point;
	}
	
	bool BasicSensorMap::isInsideWorld(float x, float y) const
	{
		return ((x >= m_worldMin.x) && (x <= m_worldMax.x) && (y >= m_worldMin.y) && (y <= m_worldMax.y));
	}
}
