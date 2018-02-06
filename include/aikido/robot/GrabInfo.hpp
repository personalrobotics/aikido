#ifndef AIKIDO_ROBOT_GRABINFO_HPP_
#define AIKIDO_ROBOT_GRABINFO_HPP_

namespace aikido {
namespace robot {

/// To store metadata for grabbed objects
class GrabInfo
{
public:

  GrabInfo(
      dart::dynamics::BodyNodePtr bodyNode,
      const std::string &oldName,
      dart::dynamics::SkeletonPtr skeleton,
      const dart::dynamics::FreeJoint::Properties& jointProperties);


  /// Update existing properties.
  void update(
      dart::dynamics::BodyNodePtr bodyNode,
      const std::string& oldName,
      dart::dynamics::SkeletonPtr skeleton,
      const dart::dynamics::FreeJoint::Properties& jointProperties);

  /// Clears existing properties
  void clear();

  /// Returns true if grabbing an object
  bool isGrabbing();

private:

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