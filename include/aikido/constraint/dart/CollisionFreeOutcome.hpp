#ifndef AIKIDO_CONSTRAINT_DART_COLLISIONFREEOUTCOME_HPP_
#define AIKIDO_CONSTRAINT_DART_COLLISIONFREEOUTCOME_HPP_

#include <vector>
#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/Contact.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include "aikido/constraint/TestableOutcome.hpp"

namespace aikido {
namespace constraint {
namespace dart {

/// TestableOutcome derivative class intended as (optional) input to isSatisfied
/// method in CollisionFree class.
class CollisionFreeOutcome : public TestableOutcome
{
  friend class CollisionFree;

public:
  /// Documentation inherited.
  bool isSatisfied() const override;

  /// Returns a string with each pair of CollisionObject names on a separate
  /// line. Each pair is also marked as being a normal collision or self
  /// collision.
  std::string toString() const override;

  /// Clears this outcome object. Useful in the event that a CollisionFree
  /// object is passed to the isSatisfied() method of more than one constraint
  /// object.
  void clear();

  /// Return a copy of the vector storing the Contact objects from pairwise
  /// collisions.
  std::vector<::dart::collision::Contact> getPairwiseContacts() const;

  /// Return a copy of the vector storing the Contact objects from self
  /// collisions.
  std::vector<::dart::collision::Contact> getSelfContacts() const;

  /// Gets the name of a CollisionObject. The name returned is that of the
  /// corresponding BodyNode (if possible). If not, the name of the ShapeFrame
  /// is returned instead. This is a helper for toString().
  /// \param[in] object object pointer to CollisionObject we want the name of.
  std::string getCollisionObjectName(
      const ::dart::collision::CollisionObject* object) const;

protected:
  /// Holds Contact objects from pairwise collisions.
  std::vector<::dart::collision::Contact> mPairwiseContacts;

  /// Holds Contact objects from self collisions.
  std::vector<::dart::collision::Contact> mSelfContacts;
};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_COLLISIONFREEOUTCOME_HPP_
