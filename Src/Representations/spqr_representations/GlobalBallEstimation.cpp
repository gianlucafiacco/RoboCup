#include "GlobalBallEstimation.h"

GlobalBallEstimationCompressed::GlobalBallEstimationCompressed(const GlobalBallEstimation& globalBallEstimation):
    observations(globalBallEstimation.observations) {}

GlobalBallEstimationCompressed::operator GlobalBallEstimation() const
{
  GlobalBallEstimation globalBallEstimation;
  globalBallEstimation.observations = "";
  return globalBallEstimation;
}
