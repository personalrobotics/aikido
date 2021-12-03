#ifndef AIKIDO_ROBOT_IK_GRADIENTDESCENTIK_HPP_
#define AIKIDO_ROBOT_IK_GRADIENTDESCENTIK_HPP_

#include <vector>
#include <Eigen/Dense>

#include <dart/dart.hpp>

#include "aikido/constraint/Sampleable.hpp"
#include "aikido/robot/InverseKinematics.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace robot {

/// Optimization-based IK solver that uses gradient descent from random seed
// configurations.
class GradientDescentIk : public InverseKinematics
{
public:

  GradientDescentIk(
    ::dart::dynamics::MetaSkeletonPtr arm,
    ::dart::dynamics::BodyNodePtr endEffector,
    statespace::dart::ConstMetaSkeletonStateSpacePtr armSpace,
    common::RNG* rng);

  // Documentation inherited.
  std::vector<Eigen::VectorXd> getSolutions(
    const Eigen::Isometry3d& targetPose, const size_t maxSolutions) const override;
  
private:
  ::dart::dynamics::MetaSkeletonPtr mArm;
  ::dart::dynamics::BodyNodePtr mEndEffector;
  statespace::dart::ConstMetaSkeletonStateSpacePtr mArmSpace;

  std::shared_ptr<aikido::constraint::Sampleable> mSeedConfigSampleable;
  ::dart::dynamics::InverseKinematicsPtr mDartIkSolver;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_IK_GRADIENTDESCENTIK_HPP_
