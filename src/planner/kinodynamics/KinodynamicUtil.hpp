#ifndef AIKIDO_PLANNER_KINODYNAMICS_KINODYNAMICUTIL_HPP_
#define AIKIDO_PLANNER_KINODYNAMICS_KINODYNAMICUTIL_HPP_

#include <string>
#include <vector>
#include <Eigen/Core>
#include <ompl/geometric/PathGeometric.h>
#include "aikido/trajectory/Spline.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/planner/kinodynamics/dimt/DoubleIntegratorMinimumTime.h"

namespace aikido {
namespace planner {
namespace kinodynamics {

void dumpSplinePhasePlot(
    const aikido::trajectory::Spline& spline,
    const std::string& filename,
    double timeStep);

bool convertPathToSequentialStates(
    ::ompl::geometric::PathGeometric* path,
    const DIMTPtr& dimt,
    double interpolateStepSize,
    std::vector<Eigen::VectorXd>& points,
    std::vector<double>& times
    );

std::unique_ptr<aikido::trajectory::Spline> 
    convertSequentialStatesToSpline(
        const statespace::dart::MetaSkeletonStateSpacePtr& metaSkeletonStateSpace,
        std::vector<Eigen::VectorXd>& points,
        std::vector<double>& times);

} // namespace kinodynamics
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_KINODYNAMICS_KINODYNAMICUTIL_HPP_
