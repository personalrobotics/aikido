#ifndef MAGI_UTIL_RETIMETRAJECTORYPOSTPROCESSOR_HPP_
#define MAGI_UTIL_RETIMETRAJECTORYPOSTPROCESSOR_HPP_

#include <magi/util/TrajectoryPostProcessor.hpp>

namespace magi {
namespace util {

class RetimeTrajectoryPostProcessor : public TrajectoryPostProcessor {
public:
  RetimeTrajectoryPostProcessor(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
      const Eigen::VectorXd &_velocityLimits,
      const Eigen::VectorXd &_accelerationLimits);

  virtual std::unique_ptr<aikido::trajectory::Spline>
  postprocess(const aikido::trajectory::InterpolatedPtr &_inputTraj,
              aikido::common::RNG *_rng) override;

  virtual std::unique_ptr<aikido::trajectory::Spline>
  postprocess(const std::unique_ptr<aikido::trajectory::Spline> &_inputTraj,
              aikido::common::RNG *_rng) override;

private:
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mSpace;

  const Eigen::VectorXd mVelocityLimits;
  const Eigen::VectorXd mAccelerationLimits;
};

} // util
} // magi

#endif // MAGI_UTIL_RETIMETRAJECTORYPOSTPROCESSOR_HPP_
