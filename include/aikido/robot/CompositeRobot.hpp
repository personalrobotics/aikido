#ifndef AIKIDO_ROBOT_METAROBOT_HPP_
#define AIKIDO_ROBOT_METAROBOT_HPP_

#include <string>
#include <unordered_map>
#include <dart/dart.hpp>
#include "aikido/robot/Robot.hpp"

namespace aikido {
namespace robot {

/// A base class for a meta robot which has a list of robots
class CompositeRobot : public Robot
{
public:
  CompositeRobot(
      const std::string& name,
      dart::dynamics::MetaSkeletonPtr robot,
      statespace::dart::MetaSkeletonStateSpacePtr statespace,
      bool simulation,
      aikido::common::RNG::result_type rngSeed,
      std::unique_ptr<control::TrajectoryExecutor> trajectoryExecutor,
      std::unique_ptr<planner::parabolic::ParabolicTimer> retimer,
      std::unique_ptr<planner::parabolic::ParabolicSmoother> smoother,
      std::unordered_map<std::string, std::unique_ptr<Robot>> robots);

  virtual ~CompositeRobot() = default;

  /// \return the named robot or nullptr if it doesn't exist
  std::unique_ptr<Robot> getRobot(const std::string& name);

protected:
  std::unordered_map<std::string, std::unique_ptr<Robot>> mRobots;
};

} // namespace robot
} // namespace aikido

#endif
