#ifndef AIKIDO_ROBOT_GRABMETADATA_HPP_
#define AIKIDO_ROBOT_GRABMETADATA_HPP_

#include <dart/dynamics/dynamics.hpp>

namespace aikido {
namespace robot {

/// Stores metadata for grabbed objects.
struct GrabMetadata
{
  /// Constructor.
  // \param[in] bodyNode BodyNode of the grabbed object.
  // \param[in] oldName Name of the grabbed object.
  // \param[in] parentSkeleton Parent skeleton of bodyNode.
  // \param[in] jointProperties Joint properties of the grabbed object.
  GrabMetadata(
      dart::dynamics::BodyNodePtr bodyNode,
      const std::string& oldName,
      dart::dynamics::SkeletonPtr parentSkeleton,
      const dart::dynamics::FreeJoint::Properties& jointProperties);

  // BodyNode of grabbed object
  dart::dynamics::BodyNodePtr mBodyNode;

  // Old name of grabbed object
  std::string mOldName;

  // Parent skeleton of bodyNode
  dart::dynamics::SkeletonPtr mParentSkeleton;

  // Original joint properties
  dart::dynamics::FreeJoint::Properties mJointProperties;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_GRABMETADATA_HPP_
