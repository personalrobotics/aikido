#ifndef AIKIDO_PLANNER_DART_SINGLEPROBLEMPLANNER_HPP_
#define AIKIDO_PLANNER_DART_SINGLEPROBLEMPLANNER_HPP_

#include "aikido/planner/SingleProblemPlanner.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Base planner class for all DART single problem planners. Avoids copying
/// MetaSkeleton related code a bunch of times.
template <typename Derived, typename ProblemT>
class SingleProblemPlanner
  : public planner::SingleProblemPlanner<Derived, ProblemT>
{
public:
  /// Constructor
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] metaSkeleton MetaSkeleton to use for planning.
  SingleProblemPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton);

  /// Return this planner's MetaSkeletonStateSpace.
  statespace::dart::ConstMetaSkeletonStateSpacePtr getMetaSkeletonStateSpace()
      const;

  /// Return this planner's MetaSkeleton.
  ::dart::dynamics::MetaSkeletonPtr getMetaSkeleton();

protected:
  /// Stores stateSpace pointer as a ConstMetaSkeletonStateSpacePtr. Prevents
  /// use of an expensive dynamic cast.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// MetaSkeleton to use for planning.
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#include "aikido/planner/dart/detail/SingleProblemPlanner-impl.hpp"

#endif // AIKIDO_PLANNER_DART_SINGLEPROBLEMPLANNER_HPP_
