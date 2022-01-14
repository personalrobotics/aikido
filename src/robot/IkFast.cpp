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
  : mArm(arm)
  , mEndEffector(endEffector)
  , mArmSpace(armSpace)
{
  // Do nothing.
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