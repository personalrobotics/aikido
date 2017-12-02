#include "aikido/planner/optimization/Function.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
void Function::setVariable(const VariablePtr& variableToClone)
{
  if (!isCompatible(variableToClone))
    throw std::invalid_argument("Invalid variable for this function.");

  mVariable = variableToClone->clone();
}

//==============================================================================
bool TrajectoryFunction::isCompatible(const VariablePtr& variable) const
{
  return variable->isTrajectoryVariable();
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
