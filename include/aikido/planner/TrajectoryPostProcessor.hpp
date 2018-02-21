#ifndef AIKIDO_PLANNER_TRAJECTORYPOSTPROCESSOR_HPP_
#define AIKIDO_PLANNER_TRAJECTORYPOSTPROCESSOR_HPP_

#include "aikido/common/RNG.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {

class TrajectoryPostProcessor
{
public:
  /// \param _inputTraj The untimed trajectory for the arm to process.
  /// \param _rng Random number generator.
  /// \param _constraint Must be satisfied after prcoessing.
  virtual std::unique_ptr<aikido::trajectory::Spline> postprocess(
      const aikido::trajectory::Interpolated& _inputTraj,
      const aikido::common::RNG& _rng,
      const aikido::constraint::TestablePtr& _constraint)
      = 0;

  /// \param _inputTraj The untimed *spline* trajectory for the arm to process.
  /// \param _rng Random number generator.
  /// \param _constraint Must be satisfied after prcoessing.
  virtual std::unique_ptr<aikido::trajectory::Spline> postprocess(
      const trajectory::Spline& _inputTraj,
      const aikido::common::RNG& _rng,
      const aikido::constraint::TestablePtr& _constraint)
      = 0;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_TRAJECTORYPOSTPROCESSOR_HPP_
