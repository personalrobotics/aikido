#ifndef MAGI_UTIL_TRAJECTORYPOSTPROCESSOR_HPP_
#define MAGI_UTIL_TRAJECTORYPOSTPROCESSOR_HPP_

#include <aikido/common/RNG.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <magi/action/Action.hpp>

namespace magi {
namespace util {

class TrajectoryPostProcessor {
public:
  /// \param _inputTraj The untimed trajectory for the arm to process.
  /// \param _collisionTestable Testable constraint to check for collision.
  /// \param _rng Random number generator.
  virtual std::unique_ptr<aikido::trajectory::Spline>
  postprocess(const aikido::trajectory::InterpolatedPtr &_inputTraj,
              aikido::common::RNG *_rng) = 0;

  /// \param _inputTraj The untimed *Spline* trajectory for the arm to process.
  /// \param _collisionTestable Testable constraint to check for collision.
  /// \param _rng Random number generator.
  virtual std::unique_ptr<aikido::trajectory::Spline>
  postprocess(const std::unique_ptr<aikido::trajectory::Spline> &_inputTraj,
              aikido::common::RNG *_rng) = 0;
};

} // util
} // magi

#endif // MAGI_UTIL_TRAJECTORYPOSTPROCESSOR_HPP_
