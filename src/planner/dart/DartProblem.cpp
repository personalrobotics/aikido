#include "aikido/planner/dart/DartProblem.hpp"


namespace aikido {
namespace planner {
namespace dart {
//==============================================================================
DartProblem::DartProblem(
      statespace::ConstStateSpacePtr stateSpace,
      constraint::ConstTestablePtr constraint)
: Problem(stateSpace, constraint)
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<Problem> DartProblem::clone(
      const statespace::dart::ConstMetaSkeletonPtr& metaSkeleton) const
{
  mMetaSkeleton = metaSkeleton;
}

} // namespace dart
} // namespace planner
} // namespace aikido
