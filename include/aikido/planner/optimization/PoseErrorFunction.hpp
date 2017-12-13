#ifndef AIKIDO_PLANNER_OPTIMIZATION_POSEERRORFUNCTION_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_POSEERRORFUNCTION_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/planner/optimization/TrajectoryFunction.hpp"
#include "aikido/planner/optimization/TrajectoryVariable.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class PoseErrorFunction : public TrajectoryFunction
{
public:
  PoseErrorFunction() = default;

  ~PoseErrorFunction() = default;

  std::shared_ptr<Function> clone() const override;

  double eval(const Eigen::VectorXd& x) override;

protected:
  statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  std::shared_ptr<TrajectoryVariable> mTrajectory;

  double mTargetTime;

  Eigen::Isometry3d mTargetPose;
  // TODO(JS): Add aligned setting

  // TODO(JS): Change to end-effector
  dart::dynamics::BodyNodePtr mTargetBodyNode;
};

// class DiscreteTimeCollisionFunction : public Function
//{
// public:
//  DiscreteTimeCollisionFunction() = default;

//  virtual ~DiscreteTimeCollisionFunction() = default;

//  double eval(const Eigen::VectorXd& x) override;

// protected:
//  std::shared_ptr<TrajectoryVariable> mTrajectory;

// private:
//};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_POSEERRORFUNCTION_HPP_
