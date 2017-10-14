#ifndef AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONFUNCTION_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONFUNCTION_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/planner/optimization/TrajectoryOptimizationVariables.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace optimization {

// TODO(JS): Rename to something like TrajectoryOptimizationFunction
class OptimizationFunction : public dart::optimizer::Function
{
public:
  OptimizationFunction() = default;

  ~OptimizationFunction() = default;

  double eval(const Eigen::VectorXd& x) override;

protected:
  std::shared_ptr<TrajectoryOptimizationVariables> mTrajectory;

  const std::shared_ptr<statespace::dart::MetaSkeletonStateSpace>& mStateSpace;

  std::shared_ptr<TrajectoryOptimizationVariables> mVariables;

  const statespace::StateSpace::State* mStartState;

  const statespace::StateSpace::State* mGoalState;

private:
};

class PoseErrorFunction : public dart::optimizer::Function
{
public:
  PoseErrorFunction() = default;

  ~PoseErrorFunction() = default;

  double eval(const Eigen::VectorXd& x) override;

protected:
  statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  std::shared_ptr<TrajectoryOptimizationVariables> mTrajectory;

  double mTargetTime;

  Eigen::Isometry3d mTargetPose;

  // TODO(JS): Change to end-effector
  dart::dynamics::BodyNodePtr mTargetBodyNode;

private:
};

class DiscreteTimeCollisionFunction : public OptimizationFunction
{
public:
  DiscreteTimeCollisionFunction() = default;

  virtual ~DiscreteTimeCollisionFunction() = default;

  double eval(const Eigen::VectorXd& x) override;

protected:
  std::shared_ptr<TrajectoryOptimizationVariables> mTrajectory;

private:
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONTRAJECTORY_HPP_
