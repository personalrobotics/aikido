#include "aikido/planner/optimization/PoseErrorFunction.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
std::shared_ptr<Function> PoseErrorFunction::clone() const
{
  return std::make_shared<PoseErrorFunction>();
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
