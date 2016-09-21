#pragma once

#include <map>

namespace PTracking
{
	/**
	 * @class PriorityBuffer
	 * 
	 * @brief Class that implements a priority buffer.
	 */
	template<typename T, class Numeric = float> class PriorityBuffer
	{
		private:
			/**
			 * @brief priority buffer.
			 */
			std::multimap<Numeric,T> m_data;
			
			/**
			 * @brief maximum size of the priority buffer.
			 */
			unsigned int m_maxindex;
			
		public:
			/**
			 * @brief Constructor that takes the maximum size of the priority buffer.
			 * 
			 * It initializes the priority buffer with the maximum size given in input.
			 * 
			 * @param m maximum size of the priority buffer.
			 */
			PriorityBuffer(unsigned int m)
			{
				m_maxindex = m;
				
				reset();
			}
			
			/**
			 * @brief Practical redefinition of a const_iterator to \a std::map<Numeric,T>::const_iterator.
			 */
			typedef typename std::map<Numeric,T>::const_iterator const_iterator;
			
			/**
			 * @brief Function that returns the first element of the priority buffer.
			 * 
			 * @return an iterator to the first element of the priority buffer.
			 */
			const_iterator begin() const
			{
				return m_data.begin();
			}
			
			/**
			 * @brief Function that returns the last element of the priority buffer.
			 * 
			 * @return an iterator to the last element of the priority buffer.
			 */
			const_iterator end() const
			{
				return m_data.end();
			}
			
			/**
			 * @brief Function that finds an element of the priority buffer, if present.
			 * 
			 * @param t element that we want to find.
			 * 
			 * @return an iterator to the element, if present, or \a multimap::end otherwise.
			 */
			const_iterator find(Numeric t)
			{
				return m_data.find(t);
			}
			
			/**
			 * @brief Function that find the maximum element of the priority buffer.
			 * 
			 * @param f reference to the element where to write the result.
			 * 
			 * @return \b true if succeeded, \b false otherwise.
			 */
			bool max(T& f)
			{
				if (m_data.size() > 0)
				{
					f = (--(m_data.end()))->second;
					
					return true;
				}
				
				return false;
			}
			
			/**
			 * @brief Function that find the minimum element of the priority buffer.
			 * 
			 * @param f reference to the element where to write the result.
			 * 
			 * @return \b true if succeeded, \b false otherwise.
			 */
			bool min(T& f)
			{
				if (m_data.size() > 0)
				{
					f = m_data.begin()->second;
					
					return true;
				}
				
				return false;
			}
			
			/**
			 * @brief Function that inserts a new element in the priority buffer.
			 * 
			 * @param t reference to the element to be pushed.
			 * 
			 * @return \b true if succeeded, \b false otherwise.
			 */
			bool push(const T& t)
			{
				if (m_data.size() < m_maxindex)
				{
					m_data.insert(std::make_pair(t.weight,t));
					
					return true;
				}
				else
				{
					const_iterator it = m_data.find(t.weight);
					
					if (it == m_data.end())
					{
						if (t.weight > it->first)
						{
							it--;
							m_data.erase(it->first);
							m_data.insert(std::make_pair(t.weight,t));
							
							return true;
						}
					}
				}
				
				return false;
			}
			
			/**
			 * @brief Function that clears the priority buffer.
			 */
			void reset()
			{
				m_data.clear();
			}
			
			/**
			 * @brief Function that returns the current size of the priority buffer.
			 * 
			 * @return the current size of the priority buffer.
			 */
			std::size_t size()
			{
				return m_data.size();
			}
	};
}
