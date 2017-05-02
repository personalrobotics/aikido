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

class SmootherFeasibilityCheckerBase;

class HauserParabolicSmoother
{
public:
    HauserParabolicSmoother(aikido::constraint::TestablePtr testable,
                            double timelimit = 3.0,
                            double blendRadius = 0.5,
                            int blendIterations = 4,
                            double tolerance = 1e-2);

    bool doShortcut(ParabolicRamp::DynamicPath& dynamicPath,
                    aikido::util::RNG::result_type _rngSeed = std::random_device{}());

    bool doBlend(ParabolicRamp::DynamicPath& dynamicPath);

protected:
    bool tryBlend(ParabolicRamp::DynamicPath& dynamicPath,
                  int attempt, double dtShortcut);

    bool needsBlend(ParabolicRamp::ParabolicRampND const &rampNd);

private:
    double blendRadius_;
    int    blendIterations_;
    double timelimit_;
    double tolerance_;

    std::shared_ptr<SmootherFeasibilityCheckerBase>        checkerBase_;
    std::shared_ptr<ParabolicRamp::RampFeasibilityChecker> feasibilityChecker_;
};

} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_PARABOLIC_SMOOTHER_HELPER_HPP_
