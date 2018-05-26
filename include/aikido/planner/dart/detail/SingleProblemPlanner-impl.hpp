#ifndef AIKIDO_PLANNER_DART_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_
#define AIKIDO_PLANNER_DART_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_

#include "aikido/planner/dart/SingleProblemPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
template <typename Derived, typename ProblemT>
SingleProblemPlanner<Derived, ProblemT>::SingleProblemPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : planner::SingleProblemPlanner<Derived, ProblemT>(stateSpace)
  , mMetaSkeletonStateSpace(std::move(stateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
{
  // Do nothing
}

//==============================================================================
template <typename Derived, typename ProblemT>
statespace::dart::ConstMetaSkeletonStateSpacePtr
SingleProblemPlanner<Derived, ProblemT>::getMetaSkeletonStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
template <typename Derived, typename ProblemT>
::dart::dynamics::MetaSkeletonPtr
SingleProblemPlanner<Derived, ProblemT>::getMetaSkeleton()
{
  return mMetaSkeleton;
}

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_
