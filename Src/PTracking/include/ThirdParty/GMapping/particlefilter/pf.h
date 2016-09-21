#pragma once

#include "../utils/gvalues.h"
#include <stdlib.h>
#include <utility>
#include <vector>

using namespace std;

typedef pair<uint,uint> UIntPair;

template <class OutputIterator, class Iterator> float toNormalForm(OutputIterator& out, const Iterator & begin, const Iterator & end)
{
	float lmax = -FLT_MAX;
	
	for (Iterator it = begin; it != end; it++) lmax = lmax>((float)(*it)) ? lmax : (float)(*it);
	
	// Convert to raw form.
	for (Iterator it = begin; it != end; it++)
	{
		*out = exp((float)(*it) - lmax);
		out++;
	}
	
	return lmax;
}

template <class OutputIterator, class Iterator,	class Numeric> void toLogForm(OutputIterator& out, const Iterator& begin, const Iterator& end, Numeric lmax)
{
	// Determine the maximum.
	for (Iterator it = begin; it != end; it++)
	{
		*out = log((Numeric)(*it)) - lmax;
		out++;
	}
	
	return lmax;
}

template <class WeightVector> void resample(vector<int>& indexes, const WeightVector& weights, unsigned int nparticles = 0)
{
	float cweight = 0;
	
	// Compute the cumulative weights.
	unsigned int n = 0;
	
	for (typename WeightVector::const_iterator it = weights.begin(); it != weights.end(); ++it)
	{
		cweight += (float)*it;
		n++;
	}
	
	if (nparticles > 0) n = nparticles;
	
	// Compute the interval.
	float interval = cweight / n;
	
	// Compute the initial target weight.
	float target = interval * ::drand48();
	
	// Compute the resampled indexes.
	cweight = 0;
	indexes.resize(n);
	
	n = 0;
	unsigned int i = 0;
	
	for (typename WeightVector::const_iterator it = weights.begin(); it != weights.end(); ++it, ++i)
	{
		cweight += (float)* it;
		
		while (cweight > target)
		{
			indexes[n++] = i;
			target += interval;
		}
	}
}

template <typename WeightVector> void normalizeWeights(WeightVector& weights, unsigned int size, float minWeight)
{
	float wmin = FLT_MAX;
	float wmax = -FLT_MAX;
	
	for (uint i = 0; i < size; i++)
	{
		wmin = wmin < weights[i] ? wmin : weights[i];
		wmax = wmax > weights[i] ? wmax : weights[i];
	}
	
	float min_normalized_value = log(minWeight);
	float max_normalized_value = log(1.);
	float dn = max_normalized_value - min_normalized_value;
	float dw = wmax - wmin;
	
	if (dw == 0) dw=1;
	
	float scale = dn / dw;
	float offset = -wmax * scale;
	
	for (uint i = 0; i < size; i++)
	{
		float w = weights[i];
		
		w = scale * w + offset;
		weights[i] = exp(w);
	}
}

template <typename Vector> void repeatIndexes(Vector& dest, const vector<int>& indexes, const Vector& particles)
{
	dest.resize(indexes.size());
	
	unsigned int i = 0;
	
	for (vector<int>::const_iterator it = indexes.begin(); it != indexes.end(); ++it)
	{
		dest[i] = particles[*it];
		i++;
	}
}

template <typename Vector> void repeatIndexes(Vector& dest, const vector<int>& indexes2, const Vector& particles, const vector<int>& indexes)
{
	dest = particles;
	
	unsigned int i = 0;
	
	for (vector<int>::const_iterator it = indexes2.begin(); it != indexes2.end(); ++it)
	{
		dest[indexes[i]] = particles[*it];
		i++;
	}
}

template <class Iterator> float neff(const Iterator& begin, const Iterator& end)
{
	float sum = 0;
	
	for (Iterator it = begin; it != end; ++it) sum += *it;
	
	float cum = 0;
	
	for (Iterator it = begin; it != end; ++it)
	{
		float w = *it / sum;
		cum += w * w;
	}
	
	return 1. / cum;
}

template <class OutputIterator, class Iterator> void rle(OutputIterator& out, const Iterator& begin, const Iterator& end)
{
	unsigned int current = 0;
	unsigned int count = 0;
	
	for (Iterator it = begin; it != end; it++)
	{
		if (it == begin)
		{
			current = *it;
			count = 1;
			
			continue;
		}
		
		if (((uint)*it) == current) count++;
		
		if (((uint)*it) != current)
		{
			*out = make_pair(current,count);
			out++;
			current = *it;
			count = 1;
		}
	}
	
	if (count > 0) *out=make_pair(current,count);
	
	out++;
}
