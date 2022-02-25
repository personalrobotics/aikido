#ifndef AIKIDO_ROBOT_IKFAST_HPP_
#define AIKIDO_ROBOT_IKFAST_HPP_

#include <vector>

#include <Eigen/Dense>
#include <dart/dart.hpp>

#include "aikido/robot/InverseKinematics.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace robot {

/// Analytic IK solver that uses pre-compiled IkFast binary.
class IkFast : public InverseKinematics
{
public:
  IkFast(
      ::dart::dynamics::MetaSkeletonPtr arm,
      ::dart::dynamics::BodyNodePtr endEffector,
      statespace::dart::ConstMetaSkeletonStateSpacePtr armSpace,
      const std::string& binaryPath);

  // Documentation inherited.
  std::vector<Eigen::VectorXd> getSolutions(
      const Eigen::Isometry3d& targetPose,
      const size_t maxSolutions) const override;

private:
  ::dart::dynamics::MetaSkeletonPtr mArm;
  ::dart::dynamics::BodyNodePtr mEndEffector;
  statespace::dart::ConstMetaSkeletonStateSpacePtr mArmSpace;

  dart::dynamics::SharedLibraryIkFast* mIkFastSolverPtr;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_IKFAST_HPP_
