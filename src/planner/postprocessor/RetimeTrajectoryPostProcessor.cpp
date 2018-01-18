#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/postprocessor/RetimeTrajectoryPostProcessor.hpp>

namespace aikido {
namespace planner {
namespace postprocessor {

RetimeTrajectoryPostProcessor::RetimeTrajectoryPostProcessor(
    aikido::statespace::StateSpacePtr _space,
    const Eigen::VectorXd& _velocityLimits,
    const Eigen::VectorXd& _accelerationLimits)
  : mSpace{_space}
  , mVelocityLimits{_velocityLimits}
  , mAccelerationLimits{_accelerationLimits}
{
  // Do nothing
}

std::unique_ptr<aikido::trajectory::Spline>
RetimeTrajectoryPostProcessor::postprocess(
    const aikido::trajectory::InterpolatedPtr& _inputTraj,
    aikido::common::RNG* _rng)
{
  using aikido::planner::parabolic::computeParabolicTiming;

  return computeParabolicTiming(
      *_inputTraj, mVelocityLimits, mAccelerationLimits);
}

} // postprocessor
} // planner
} // aikido
