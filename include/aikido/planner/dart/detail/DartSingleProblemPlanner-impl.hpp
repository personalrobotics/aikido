#ifndef AIKIDO_PLANNER_DETAIL_DARTSINGLEPROBLEMPLANNER_IMPL_HPP_
#define AIKIDO_PLANNER_DETAIL_DARTSINGLEPROBLEMPLANNER_IMPL_HPP_

#include "aikido/planner/dart/DartSingleProblemPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
template <typename Derived, typename ProblemT>
DartSingleProblemPlanner<Derived, ProblemT>::DartSingleProblemPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : SingleProblemPlanner<Derived, ProblemT>(stateSpace)
  , mMetaSkeletonStateSpace(std::move(stateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
{
  // Do nothing
}

//==============================================================================
template <typename Derived, typename ProblemT>
statespace::dart::ConstMetaSkeletonStateSpacePtr
DartSingleProblemPlanner<Derived, ProblemT>::getMetaSkeletonStateSpace()
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
template <typename Derived, typename ProblemT>
::dart::dynamics::MetaSkeletonPtr
DartSingleProblemPlanner<Derived, ProblemT>::getMetaSkeleton()
{
  return mMetaSkeleton;
}

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DETAIL_DARTSINGLEPROBLEMPLANNER_IMPL_HPP_
