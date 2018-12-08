#include "aikido/constraint/dart/TestableIntersection.hpp"

namespace aikido {
namespace constraint {
namespace dart {

TestableIntersection::TestableIntersection(
      statespace::ConstStateSpacePtr _stateSpace,
      std::vector<ConstTestablePtr> _constraints)
: constraint::TestableIntersection(_stateSpace, _constraints)
{
  // Do nothing
}
//==============================================================================
TestablePtr TestableIntersection::clone(
    ::dart::collision::CollisionDetectorPtr collisionDetector,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton) const
{
  std::cout << "Clone TestableIntersection constraint" << std::endl;

  std::vector<ConstTestablePtr> clonedConstraints;
  clonedConstraints.reserve(mConstraints.size());

  for (const auto& constraint : mConstraints)
  {
    auto dartConstraint = std::dynamic_pointer_cast<const DartConstraint>(constraint);
    if (dartConstraint)
    {
      clonedConstraints.emplace_back(
          dartConstraint->clone(collisionDetector, metaSkeleton));
    }
    else
    {
    clonedConstraints.emplace_back(constraint);
    }
  }

  return std::make_shared<TestableIntersection>(mStateSpace, clonedConstraints);
}

} // namespace dart
} // namespace constraint
} // namespace aikido
