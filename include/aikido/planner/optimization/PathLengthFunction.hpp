#ifndef AIKIDO_PLANNER_OPTIMIZATION_PATHLENGTHFUNCTION_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_PATHLENGTHFUNCTION_HPP_

#include <dart/optimizer/Function.hpp>
#include "aikido/planner/optimization/Function.hpp"
#include "aikido/planner/optimization/TrajectoryVariable.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class PathLengthFunction : public TrajectoryFunction
{
public:
  /// Constructor
  explicit PathLengthFunction(const distance::DistanceMetricPtr& distanceMetric)
    : mDistanceMetric(distanceMetric)
  {
    // Do nothing
  }

  // Documentation inherited.
  double eval(const Eigen::VectorXd& x) override
  {
    mVariable->setValue(x);

    const auto& trajectory = getTrajectory();
    const auto arcLength = trajectory.computeArcLength(mDistanceMetric.get());

    return arcLength;
  }

  // TODO(JS): Add setter/getter for distance metric

protected:
  distance::DistanceMetricPtr mDistanceMetric;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_PATHLENGTHFUNCTION_HPP_
