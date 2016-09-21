#include "RobotPoseSpqrFiltered.h"

RobotPoseSpqrFilteredCompressed::RobotPoseSpqrFilteredCompressed(const RobotPoseSpqrFiltered& robotPoseSpqrFiltered)
: x(robotPoseSpqrFiltered.x), y(robotPoseSpqrFiltered.y), theta(robotPoseSpqrFiltered.theta) {}

RobotPoseSpqrFilteredCompressed::operator RobotPoseSpqrFiltered() const
{
  RobotPoseSpqrFiltered robotPoseSpqrFiltered;
  robotPoseSpqrFiltered.x = 0.0;
  robotPoseSpqrFiltered.y = 0.0;
  robotPoseSpqrFiltered.theta = 0.0;
  return robotPoseSpqrFiltered;
}
