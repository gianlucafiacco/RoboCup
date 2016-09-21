#include "SpqrDWKcombiner.h"

SpqrDWKcombinerCompressed::SpqrDWKcombinerCompressed(const SpqrDWKcombiner& spqrDWKcombiner):
    explored_clusters_centroids(spqrDWKcombiner.explored_clusters_centroids) {}

SpqrDWKcombinerCompressed::operator SpqrDWKcombiner() const
{
  SpqrDWKcombiner spqrDWKcombiner;
  spqrDWKcombiner.explored_clusters_centroids = std::vector<Vector2f >();
  return spqrDWKcombiner;
}
