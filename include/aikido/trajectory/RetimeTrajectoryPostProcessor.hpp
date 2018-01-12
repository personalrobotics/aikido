#ifndef AIKIDO_TRAJECTORY_RETIMETRAJECTORYPOSTPROCESSOR_HPP_
#define AIKIDO_TRAJECTORY_RETIMETRAJECTORYPOSTPROCESSOR_HPP_

#include "TrajectoryPostProcessor.hpp"

namespace aikido {
namespace trajectory {

class RetimeTrajectoryPostProcessor : public TrajectoryPostProcessor {
public:
  RetimeTrajectoryPostProcessor(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
      const Eigen::VectorXd &_velocityLimits,
      const Eigen::VectorXd &_accelerationLimits);

  virtual std::unique_ptr<aikido::trajectory::Spline>
  postprocess(const aikido::trajectory::InterpolatedPtr &_inputTraj,
              aikido::common::RNG *_rng) override;

private:
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mSpace;

  const Eigen::VectorXd mVelocityLimits;
  const Eigen::VectorXd mAccelerationLimits;
};

} // trajectory
} // aikido

#endif // AIKIDO_TRAJECTORY_RETIMETRAJECTORYPOSTPROCESSOR_HPP_
