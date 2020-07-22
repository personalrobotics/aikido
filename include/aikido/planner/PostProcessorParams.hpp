#ifndef AIKIDO_PLANNER_POSTPROCESSORPARAMS_HPP_
#define AIKIDO_PLANNER_POSTPROCESSORPARAMS_HPP_

#include <chrono>

#include "aikido/planner/kunzretimer/KunzRetimer.hpp"
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"

namespace aikido {
namespace planner {

/// Trajectory postprocessing parameters. This struct can be used with the
/// `ConcreteRobot` class, and let's the user specify the exact processor and
//  and parameters to use.
template <typename T>
struct PostProcessorParams
{
  static_assert(
      std::is_base_of<aikido::planner::TrajectoryPostProcessor, T>::value,
      "T must derive from aikido::planner::TrajectoryPostProcessor");

public:
  /// Constructor.
  PostProcessorParams(typename T::Params params = T::Params()) : mParams(params)
  {
    // Do nothing.
  }

  typename T::Params getParams()
  {
    return mParams;
  };

private:
  typename T::Params mParams;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_POSTPROCESSORPARAMS_HPP_
