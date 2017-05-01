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

class SmootherFeasibilityChecker : public ParabolicRamp::FeasibilityCheckerBase
{
public:
    SmootherFeasibilityChecker(aikido::constraint::TestablePtr testable);

    virtual bool ConfigFeasible(ParabolicRamp::Vector const &x);
    virtual bool SegmentFeasible(ParabolicRamp::Vector const &a,
                                 ParabolicRamp::Vector const &b);
private:
    aikido::constraint::TestablePtr testable_;
};

class HauserParabolicSmoother
{
public:
    HauserParabolicSmoother(aikido::constraint::TestablePtr testable,
                            double timelimit = 3.0,
                            bool useVelocity = true,
                            double blendRadius = 0.5,
                            int blendIterations = 4,
                            double tolerance = 1e-2);


    bool doShortcut(ParabolicRamp::DynamicPath* dynamicPath);

    bool doBlend(ParabolicRamp::DynamicPath* dynamicPath);

protected:
    bool tryBlend(ParabolicRamp::DynamicPath* dynamicPath,
                  int attempt, double dtShortcut);



    bool needsBlend(ParabolicRamp::ParabolicRampND const &rampNd);

private:
    double blendRadius_;
    int    blendIterations_;
    double timelimit_;
    bool useVelocity_;
    double tolerance_;

    ParabolicRamp::RampFeasibilityChecker feasibilityChecker_;
};

} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_PARABOLIC_SMOOTHER_HELPER_HPP_
