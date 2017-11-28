#ifndef AIKIDO_CONSTRAINT_COLLISIONFREEOUTCOME_HPP_
#define AIKIDO_CONSTRAINT_COLLISIONFREEOUTCOME_HPP_

#include <sstream>
#include <vector>
#include "TestableOutcome.hpp"

namespace aikido {
namespace constraint {

class CollisionFreeOutcome : public TestableOutcome
{
public:
  /// Documentation inherited.
  bool isSatisfied() const override;

  /// Returns a string with the name of each colliding BodyNode on a separate
  /// line. Each BodyNode is also marked as being a normal collision or self
  /// collision.
  std::string toString() const override;

  /// Mark a colliding BodyNode in a normal collision. Used by CollisionFree
  /// to modify this object.
  /// \param bodyNodeName name of body node collided with.
  void markCollisionBodyNode(const std::string& bodyNodeName);

  /// Mark a colliding BodyNode in a self collision. Used by CollisionFree to
  /// modify this object.
  /// \param bodyNodeName name of body node collided with during self collision.
  void markSelfCollisionBodyNode(const std::string& bodyNodeName);

private:
  /// Holds names of colliding BodyNodes.
  std::vector<std::string> mCollisionBodyNodes;

  /// Holds names of self-colliding BodyNodes.
  std::vector<std::string> mSelfCollisionBodyNodes;
};

} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_COLLISIONFREEOUTCOME_HPP_
