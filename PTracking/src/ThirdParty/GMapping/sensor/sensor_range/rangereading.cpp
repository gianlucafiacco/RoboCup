#include "rangereading.h"
#include <ThirdParty/GMapping/utils/gvalues.h>
#include <assert.h>
#include <sys/types.h>
#include <limits>

using namespace std;

namespace GMapping
{
	RangeReading::RangeReading(const RangeSensor* rs, float time) : SensorReading(rs,time) {;}
	
	RangeReading::RangeReading(unsigned int n_beams, const float* d, const RangeSensor* rs, float time) : SensorReading(rs,time)
	{
		assert(n_beams == rs->beams().size());
		
		resize(n_beams);
		
		for (unsigned int i = 0; i < size(); i++) (*this)[i] = d[i];
	}
	
	RangeReading::~RangeReading() {;}
	
	unsigned int RangeReading::activeBeams(float density) const
	{
		if (density == 0.) return size();
		
		Point lastPoint(0,0);
		uint suppressed = 0;
		int ab = 0;
		
		for (unsigned int i = 0; i < size(); i++)
		{
			const RangeSensor* rs = dynamic_cast<const RangeSensor*>(getSensor());
			assert(rs);
			
			Point lp(cos(rs->beams()[i].pose.theta) * (*this)[i],sin(rs->beams()[i].pose.theta) * (*this)[i]);
			Point dp = lastPoint - lp;
			
			float distance = sqrt(dp * dp);
			
			if (distance < density) suppressed++;
			else
			{
				lastPoint = lp;
				ab++;
			}
		}
		
		return ab;
	}
	
	vector<Point> RangeReading::cartesianForm(float maxRange) const
	{
		const RangeSensor* rangeSensor = dynamic_cast<const RangeSensor*>(getSensor());
		assert(rangeSensor && rangeSensor->beams().size());
		
		uint m_beams = static_cast<unsigned int>(rangeSensor->beams().size());
		
		vector<Point> cartesianPoints(m_beams);
		float px, py, ps, pc;
		
		px = rangeSensor->getPose().x;
		py = rangeSensor->getPose().y;
		ps = sin(rangeSensor->getPose().theta);
		pc = cos(rangeSensor->getPose().theta);
		
		for (unsigned int i = 0; i < m_beams; i++)
		{
			const float& rho = (*this)[i];
			const float& s = rangeSensor->beams()[i].s;
			const float& c = rangeSensor->beams()[i].c;
			
			if (rho >= maxRange) cartesianPoints[i] = Point(0,0);
			else
			{
				Point p = Point(rangeSensor->beams()[i].pose.x + c * rho,rangeSensor->beams()[i].pose.y + s * rho);
				
				cartesianPoints[i].x = px + pc * p.x - ps * p.y;
				cartesianPoints[i].y = py + ps * p.x + pc * p.y;
			}
		}
		
		return cartesianPoints;
	}
	
	unsigned int RangeReading::rawView(float* v, float density) const
	{
		if (density == 0)
		{
			for (unsigned int i = 0; i < size(); i++) v[i] = (*this)[i];
		}
		else
		{
			Point lastPoint(0,0);
			uint suppressed = 0;
			
			for (unsigned int i = 0; i < size(); i++)
			{
				const RangeSensor* rs = dynamic_cast<const RangeSensor*>(getSensor());
				assert(rs);
				
				Point lp(cos(rs->beams()[i].pose.theta) * (*this)[i],sin(rs->beams()[i].pose.theta) * (*this)[i]);
				Point dp = lastPoint - lp;
				
				float distance = sqrt(dp * dp);
				
				if (distance < density)
				{
					v[i] = numeric_limits<float>::max();
					suppressed++;
				}
				else
				{
					lastPoint = lp;
					v[i] = (*this)[i];
				}
			}
		}
		
		return static_cast<unsigned int>(size());
	}
}
