#ifndef AIKIDO_ROBOT_GRABMETADATA_HPP_
#define AIKIDO_ROBOT_GRABMETADATA_HPP_

#include <dart/dart.hpp>
#include <dart/dynamics/dynamics.hpp>

namespace aikido {
namespace robot {

/// Stores metadata for grabbed objects.
class GrabMetadata
{
public:
  /// Constructor.
  // \param bodyNode BodyNode of the grabbed object.
  // \param oldName Name of the grabbed object.
  // \param parentSkeleton Parent skeleton of bodyNode.
  // \param joitnProperties Joint properties of the grabbed object.
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

#endif
