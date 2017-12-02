#ifndef AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONFUNCTION_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONFUNCTION_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/planner/optimization/TrajectoryVariable.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace optimization {

// TODO(JS): Rename to something like TrajectoryOptimizationFunction
class Function : public dart::optimizer::Function
{
public:
  Function() = default;

  virtual ~Function() = default;

protected:
  friend class OptimizationBasedMotionPlanning;

  void setVariable(const VariablePtr& variableToClone);

  virtual bool isCompatible(const VariablePtr& variable) const = 0;

  std::shared_ptr<Variable> mVariable;
};

using FunctionPtr = std::shared_ptr<Function>;
using ConstFunctionPtr = std::shared_ptr<const Function>;

class TrajectoryFunction : public Function
{
public:
  TrajectoryVariablePtr getTrajectoryVariable()
  {
    return std::static_pointer_cast<TrajectoryVariable>(mVariable);
  }

  ConstTrajectoryVariablePtr getTrajectoryVariable() const
  {
    return std::static_pointer_cast<TrajectoryVariable>(mVariable);
  }

  const trajectory::Trajectory& getTrajectory() const
  {
    return getTrajectoryVariable()->getTrajectory();
  }

protected:
  bool isCompatible(const VariablePtr& variable) const override;
};

class PoseErrorFunction : public dart::optimizer::Function
{
public:
  PoseErrorFunction() = default;

  ~PoseErrorFunction() = default;

  double eval(const Eigen::VectorXd& x) override;

protected:
  statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  std::shared_ptr<TrajectoryVariable> mTrajectory;

  double mTargetTime;

  Eigen::Isometry3d mTargetPose;

  // TODO(JS): Change to end-effector
  dart::dynamics::BodyNodePtr mTargetBodyNode;

private:
};

class DiscreteTimeCollisionFunction : public Function
{
public:
  DiscreteTimeCollisionFunction() = default;

  virtual ~DiscreteTimeCollisionFunction() = default;

  double eval(const Eigen::VectorXd& x) override;

protected:
  std::shared_ptr<TrajectoryVariable> mTrajectory;

private:
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONTRAJECTORY_HPP_
