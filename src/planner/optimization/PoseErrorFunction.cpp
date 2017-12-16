#include "aikido/planner/optimization/PoseErrorFunction.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
PoseErrorFunction::PoseErrorFunction(TrajectoryVariablePtr variable)
  : TrajectoryFunction(std::move(variable))
{
  // Do nothing
}

//==============================================================================
UniqueFunctionPtr PoseErrorFunction::clone() const
{
  return dart::common::make_unique<PoseErrorFunction>(mVariable);
}

//==============================================================================
double PoseErrorFunction::eval(const Eigen::VectorXd& x)
{
  auto metaSkel = mMetaSkeletonStateSpace->getMetaSkeleton();

  auto state = mMetaSkeletonStateSpace->createState();

  mTrajectory->setValue(x);
  //  mTrajectory->evaluate(mTargetTime, state);

  Eigen::Isometry3d currentPose = mTargetBodyNode->getTransform();

  Eigen::Isometry3d errorPose = mTargetPose.inverse() * currentPose;

  Eigen::Vector6d errorParams = dart::math::logMap(errorPose);

  return errorParams.norm();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
