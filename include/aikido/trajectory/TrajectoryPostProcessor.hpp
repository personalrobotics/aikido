#ifndef AIKIDO_TRAJECTORY_TRAJECTORYPOSTPROCESSOR_HPP_
#define AIKIDO_TRAJECTORY_TRAJECTORYPOSTPROCESSOR_HPP_

#include "../common/RNG.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include "../trajectory/Interpolated.hpp"
#include "../trajectory/Spline.hpp"

namespace aikido {
namespace trajectory {

class TrajectoryPostProcessor {
public:
  /// \param _inputTraj The untimed trajectory for the arm to process.
  /// \param _rng Random number generator.
  virtual std::unique_ptr<aikido::trajectory::Spline>
  postprocess(const aikido::trajectory::InterpolatedPtr &_inputTraj,
              aikido::common::RNG *_rng) = 0;
};

} // trajectory
} // aikido

#endif // AIKIDO_TRAJECTORY_TRAJECTORYPOSTPROCESSOR_HPP_
