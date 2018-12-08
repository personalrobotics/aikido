#ifndef AIKIDO_CONSTRAINT_DART_DARTCONSTRAINT_HPP_
#define AIKIDO_CONSTRAINT_DART_DARTCONSTRAINT_HPP_

#include <dart/dynamics/dynamics.hpp>
#include <dart/collision/collision.hpp>

#include "aikido/common/pointers.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace constraint {
namespace dart {

/// Abstract base class for constraints with DART components
class DartConstraint
{
public:
  // \param[in] metaSkeleton Metaskeleton with which to
  // clone this constraint.
  virtual TestablePtr clone(
      ::dart::collision::CollisionDetectorPtr collisionDetector,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton) const = 0;

};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_DARTCONSTRAINT_HPP_
