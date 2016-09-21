#pragma once

#include "../Filters/ObjectParticleFilterMultiAgent.h"
#include "../../Utils/Timestamp.h"
#include <Manfield/manifoldprocessor.h>

namespace PTracking
{
	/**
	 * @class MultiAgentProcessor
	 * 
	 * @brief Class that implements a multi agent processor.
	 */
	class MultiAgentProcessor : public manfield::ManifoldFilterProcessor
	{
		private:
			/**
			 * @brief timestamp of the last iteration.
			 */
			Timestamp timeOfLastIteration;
			
		public:
			/**
			 * @brief Empty constructor.
			 */
			MultiAgentProcessor();
			
			/**
			 * @brief Destructor.
			 */
			~MultiAgentProcessor();
			
			/**
			 * @brief Function that initializes several configuration parameters.
			 */
			void init();
			
			/**
			 * @brief Function that processes, if needed, the estimations received by the other agents between the previous and current iteration.
			 * 
			 * @param readings reference to the estimations received by the other agents between the previous and current iteration.
			 */
			void processReading(const std::vector<ObjectSensorReadingMultiAgent>& readings);
			
			/**
			 * @brief Function that invokes the predict and update step of the underlying particle filter.
			 * 
			 * @param f reference to the underlying particle filter.
			 * @param readings reference to the estimations received by the other agents between the previous and current iteration.
			 */
			void singleFilterIteration(ObjectParticleFilterMultiAgent& f, const vector<ObjectSensorReadingMultiAgent>& readings) const;
			
			/**
			 * @brief Function that checks if the processing step is needed.
			 * 
			 * @return \b true if the processing step is needed, \b false otherwise.
			 */
			bool updateNeeded() const;
			
			PARAM_SET_GET(float, updateFrequency, private, public, public)
			PARAM_SET_GET(unsigned int, nFusedParticles, private, public, public)
	};
}
