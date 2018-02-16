#include "aikido/robot/GrabMetadata.hpp"

namespace aikido {
namespace robot {

//================================================================
GrabMetadata::GrabMetadata(
    dart::dynamics::BodyNodePtr bodyNode,
    const std::string &oldName,
    dart::dynamics::SkeletonPtr parentSkeleton,
    const dart::dynamics::FreeJoint::Properties &jointProperties)
: mBodyNode(bodyNode)
, mOldName(oldName)
, mParentSkeleton(parentSkeleton)
, mJointProperties(jointProperties)
{
  // Do nothing
}

} // namespace robot
} // namespace aikido

