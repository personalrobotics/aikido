#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorTwistVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include "detail/VectorFieldPlannerExceptions.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
MoveEndEffectorTwistVectorField::MoveEndEffectorTwistVectorField(
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaskeleton,
    ::dart::dynamics::ConstBodyNodePtr bn,
    const std::vector<Eigen::Vector6d>& twistSeq,
    const std::vector<double> durationSeq,
    double positionTolerance,
    double angularTolerance,
    double maxStepSize,
    double jointLimitPadding)
  : BodyNodePoseVectorField(
        stateSpace, metaskeleton, bn, maxStepSize, jointLimitPadding)
  , mPositionTolerance(positionTolerance)
  , mAngularTolerance(angularTolerance)
  , mStartPose(bn->getTransform())
{
  if (mPositionTolerance < 0)
    throw std::invalid_argument("Position tolerance is negative");
  if (mAngularTolerance < 0)
    throw std::invalid_argument("Angular tolerance is negative");

  // create an interpolated from twistSeq and durationSeq
}

//==============================================================================
bool MoveEndEffectorTwistVectorField::evaluateCartesianVelocity(
    const Eigen::Isometry3d& pose, Eigen::Vector6d& cartesianVelocity) const
{
  using ::dart::math::logMap;
  using aikido::planner::vectorfield::computeGeodesicError;

  Eigen::Vector6d desiredTwist;

  // query a desired Twist from the trajectory
  cartesianVelocity = desiredTwist;
  return true;
}

//==============================================================================
VectorFieldPlannerStatus
MoveEndEffectorTwistVectorField::evaluateCartesianStatus(
    const Eigen::Isometry3d& pose) const
{
  // compute the error between desired twist and current twist

  return VectorFieldPlannerStatus::CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
