#include "aikido/planner/optimization/OptimizationFunction.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
double PoseErrorFunction::eval(const Eigen::VectorXd& x) const
{
  auto metaSkel = mMetaSkeletonStateSpace->getMetaSkeleton();

  auto state = mMetaSkeletonStateSpace->createState();

  mTrajectory->setVariables(x);
  mTrajectory->evaluate(mTargetTime, state);

  Eigen::Isometry3d currentPose = mTargetBodyNode->getTransform();

  Eigen::Isometry3d errorPose = mTargetPose.inverse() * currentPose;

  Eigen::Vector6d errorParams = dart::math::logMap(errorPose);

  return errorParams.norm();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
