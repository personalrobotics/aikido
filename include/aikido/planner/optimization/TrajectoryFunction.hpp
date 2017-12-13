#ifndef AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYFUNCTION_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYFUNCTION_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/planner/optimization/Function.hpp"
#include "aikido/planner/optimization/TrajectoryVariable.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class TrajectoryFunction : public Function
{
public:
  TrajectoryVariablePtr getTrajectoryVariable();

  ConstTrajectoryVariablePtr getTrajectoryVariable() const;

  const trajectory::Trajectory& getTrajectory() const;

protected:
  bool isCompatible(const Variable& variable) const override;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYFUNCTION_HPP_
