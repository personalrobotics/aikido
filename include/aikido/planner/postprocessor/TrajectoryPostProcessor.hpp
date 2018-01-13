#ifndef AIKIDO_PLANNER_POSTPROCESSOR_TRAJECTORYPOSTPROCESSOR_HPP_
#define AIKIDO_PLANNER_POSTPROCESSOR_TRAJECTORYPOSTPROCESSOR_HPP_

#include "../../common/RNG.hpp"
#include "../../trajectory/Interpolated.hpp"
#include "../../trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace postprocessor {

class TrajectoryPostProcessor
{
public:
  /// \param _inputTraj The untimed trajectory for the arm to process.
  /// \param _rng Random number generator.
  virtual std::unique_ptr<aikido::trajectory::Spline> postprocess(
      const aikido::trajectory::InterpolatedPtr& _inputTraj,
      aikido::common::RNG* _rng)
      = 0;
};

} // postprocessor
} // planner
} // aikido

#endif // AIKIDO_PLANNER_POSTPROCESSOR_TRAJECTORYPOSTPROCESSOR_HPP_
