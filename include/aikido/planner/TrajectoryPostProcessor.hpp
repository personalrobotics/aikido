#ifndef AIKIDO_PLANNER_TRAJECTORYPOSTPROCESSOR_HPP_
#define AIKIDO_PLANNER_TRAJECTORYPOSTPROCESSOR_HPP_

#include "aikido/common/RNG.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {

class TrajectoryPostProcessor
{
public:
  /// \param _inputTraj The untimed trajectory for the arm to process.
  /// \param _rng Random number generator.
  virtual std::unique_ptr<aikido::trajectory::Spline> postprocess(
      const aikido::trajectory::Interpolated& _inputTraj,
      const aikido::common::RNG* _rng)
      = 0;

  /// \param _inputTraj The untimed *spline* trajectory for the arm to process.
  /// \param _rng Random number generator.
  virtual std::unique_ptr<aikido::trajectory::Spline> postprocess(
      const trajectory::Spline& _inputTraj,
      const aikido::common::RNG* _rng)
      = 0;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_TRAJECTORYPOSTPROCESSOR_HPP_
