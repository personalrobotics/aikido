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
namespace hauserParabolicSmoother {

  bool doShortcut(ParabolicRamp::DynamicPath& dynamicPath,
                  aikido::constraint::TestablePtr testable,
                  double timelimit, double tolerance,
                  aikido::util::RNG::result_type rngSeed =
                      std::random_device{}());

  bool doBlend(ParabolicRamp::DynamicPath& dynamicPath,
               aikido::constraint::TestablePtr testable,
               double tolerance, double blendRadius,
               int blendIterations);

} // namespace hauserParabolicSmoother
} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_PARABOLIC_SMOOTHER_HELPER_HPP_
