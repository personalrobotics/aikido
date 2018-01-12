#include <aikido/trajectory/RetimeTrajectoryPostProcessor.hpp>
#include <aikido/trajectory/postprocess.hpp>

namespace aikido {
namespace trajectory {

RetimeTrajectoryPostProcessor::RetimeTrajectoryPostProcessor(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
    const Eigen::VectorXd &_velocityLimits,
    const Eigen::VectorXd &_accelerationLimits)
    : mSpace{_space}, mVelocityLimits{_velocityLimits},
      mAccelerationLimits{_accelerationLimits} {
  // Do nothing
}

std::unique_ptr<aikido::trajectory::Spline>
RetimeTrajectoryPostProcessor::postprocess(
    const aikido::trajectory::InterpolatedPtr &_inputTraj,
    aikido::common::RNG *_rng) {
  return postprocess::timeTrajectory(_inputTraj, mVelocityLimits,
                                     mAccelerationLimits);
}

} // namespace trajectory
} // namespace aikido
