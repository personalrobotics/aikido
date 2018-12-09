#ifndef AIKIDO_PLANNER_DART_DARTPLANNER_HPP_
#define AIKIDO_PLANNER_DART_DARTPLANNER_HPP_

#include <dart/dart.hpp>
#include "aikido/planner/Planner.hpp"

namespace aikido {
namespace planner {
namespace dart{


/// Base class for various planners with DART component.
class DartPlanner
{
public:
  /// Clones this planner.
  /// \param[in] rng RNG to be used for random seed
  /// \param[in] metaSkeleton MetaSkeleton to be used for cloning
  /// relevant properties.
  virtual PlannerPtr clone(
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      common::RNG* rng = nullptr) const= 0;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_DARTPLANNER_HPP_
