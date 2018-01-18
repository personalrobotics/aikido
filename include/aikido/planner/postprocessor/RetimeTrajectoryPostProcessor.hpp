#ifndef AIKIDO_PLANNER_POSTPROCESSOR_RETIMETRAJECTORYPOSTPROCESSOR_HPP_
#define AIKIDO_PLANNER_POSTPROCESSOR_RETIMETRAJECTORYPOSTPROCESSOR_HPP_

#include "aikido/planner/postprocessor/TrajectoryPostProcessor.hpp"

namespace aikido {
namespace planner {
namespace postprocessor {

class RetimeTrajectoryPostProcessor : public TrajectoryPostProcessor
{
public:
  /// Class for performing parabolic retiming on trajectories.
  /// \param _space pointer to statespace trajectories correspond to.
  /// \param _velocityLimits maximum velocity for each dimension.
  /// \param _accelerationLimits maximum acceleration for each dimension.
  RetimeTrajectoryPostProcessor(
      aikido::statespace::StateSpacePtr _space,
      const Eigen::VectorXd& _velocityLimits,
      const Eigen::VectorXd& _accelerationLimits);

  /// Performs parabolic retiming on an input trajectory.
  /// Documentation inherited.
  std::unique_ptr<aikido::trajectory::Spline> postprocess(
      const aikido::trajectory::InterpolatedPtr& _inputTraj,
      const aikido::common::RNG* _rng) override;

private:
  aikido::statespace::StateSpacePtr mSpace;

  const Eigen::VectorXd mVelocityLimits;
  const Eigen::VectorXd mAccelerationLimits;
};

} // namespace postprocessor
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_POSTPROCESSOR_RETIMETRAJECTORYPOSTPROCESSOR_HPP_
