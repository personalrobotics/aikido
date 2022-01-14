#include "aikido/robot/IkFast.hpp"

#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"

using aikido::statespace::dart::MetaSkeletonStateSaver;

namespace aikido {
namespace robot {

//==============================================================================
IkFast::IkFast(
    ::dart::dynamics::MetaSkeletonPtr arm,
    ::dart::dynamics::BodyNodePtr endEffector,
    statespace::dart::ConstMetaSkeletonStateSpacePtr armSpace,
    const std::string& binaryPath)
  : mArm(arm), mEndEffector(endEffector), mArmSpace(armSpace)
{
  dart::dynamics::EndEffector* ikFastEePtr
      = endEffector->createEndEffector("ee");

  auto ik = ikFastEePtr->createIK();

  std::vector<std::size_t> ikFastSolveDofs;
  std::vector<std::size_t> ikFastFreeDofs = {};

  // TODO (sniyaz): Add free DOF support!
  for (size_t i = 0; i < armSpace->getProperties().getNumJoints(); i++)
  {
    ikFastSolveDofs.push_back(i);
  }

  ik->setGradientMethod<dart::dynamics::SharedLibraryIkFast>(
      binaryPath, ikFastSolveDofs, ikFastFreeDofs);

  // Sets ikFast solver pointer as an instance variable.
  auto analyticIKPtr = ik->getAnalytical();
  mIkFastSolverPtr
      = dynamic_cast<dart::dynamics::SharedLibraryIkFast*>(analyticIKPtr);
}

//==============================================================================
std::vector<Eigen::VectorXd> IkFast::getSolutions(
    const Eigen::Isometry3d& targetPose, const size_t maxSolutions) const
{
  auto saver = MetaSkeletonStateSaver(
      mArm, MetaSkeletonStateSaver::Options::POSITIONS);
  DART_UNUSED(saver);

  std::vector<Eigen::VectorXd> solutions;

  // TODO.

  return solutions;
}

} // namespace robot
} // namespace aikido