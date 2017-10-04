#ifndef AIKIDO_PLANNER_PARABOLIC_SMOOTHER_HELPER_HPP_
#define AIKIDO_PLANNER_PARABOLIC_SMOOTHER_HELPER_HPP_

#include <Eigen/Dense>
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/constraint/Testable.hpp"
#include "DynamicPath.h"

namespace aikido {
namespace planner {
namespace parabolic {
namespace detail {

  bool doShortcut(ParabolicRamp::DynamicPath& dynamicPath,
                  aikido::constraint::TestablePtr testable,
                  double timelimit,
                  double checkResolution, double tolerance,
                  aikido::common::RNG& rng);

  bool doBlend(ParabolicRamp::DynamicPath& dynamicPath,
               aikido::constraint::TestablePtr testable,
               double blendRadius, int blendIterations,
               double checkResolution, double tolerance);

} // namespace detail
} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_PARABOLIC_SMOOTHER_HELPER_HPP_
