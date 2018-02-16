#include "aikido/robot/CompositeRobot.hpp"

namespace aikido {
namespace robot {

//=============================================================================
  CompositeRobot::CompositeRobot(
    const std::string& name,
    dart::dynamics::MetaSkeletonPtr robot,
    statespace::dart::MetaSkeletonStateSpacePtr statespace,
    bool simulation,
    aikido::common::RNG::result_type rngSeed,
    std::unique_ptr<control::TrajectoryExecutor> trajectoryExecutor,
    std::unique_ptr<planner::parabolic::ParabolicTimer> retimer,
    std::unique_ptr<planner::parabolic::ParabolicSmoother> smoother,
    std::unordered_map<std::string, std::unique_ptr<Robot>> robots)
  : Robot(name,
    robot,
    statespace,
    simulation,
    rngSeed,
    std::move(trajectoryExecutor),
    std::move(retimer),
    std::move(smoother))
    ,mRobots(std::move(robots))
  {
    // Do nothing
  }


} // namespace robot
} // namespace aikido

