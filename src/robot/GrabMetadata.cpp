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


//================================================================
void GrabMetadata::update(
    dart::dynamics::BodyNodePtr bodyNode,
    const std::string& oldName,
    dart::dynamics::SkeletonPtr parentSkeleton,
    const dart::dynamics::FreeJoint::Properties& jointProperties)
{
  mBodyNode = bodyNode;
  mOldName = oldName;
  mParentSKeleton = parentSkeleton;
  mJointProperties = jointProperties;
}

//===============================================================
void GrabMetadata::clear()
{
  bodyNode.reset();
  oldName = "";
  skeleton.reset();
  // TODO: reset joint properties
}


} // namespace robot
} // namespace aikido

