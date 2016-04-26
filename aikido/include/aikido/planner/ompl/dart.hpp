#ifndef AIKIDO_PLANNER_OMPL_DART_HPP_
#define AIKIDO_PLANNER_OMPL_DART_HPP_

#include <dart/dynamics/Skeleton.h>
#include <ompl/base/SpaceInformation.h>
#include <aikido/constraint/Testable.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

namespace aikido {
namespace planner {
namespace ompl {
/// Create a SpaceInformation by constructing default bounds constraints and metrics for
/// the given robot
/// \param _stateSpace The dart MetaSkeletonStateSpace to plan in
/// \param _validityConstraint A constraint describing a set of checks that
/// should be done before marking a state as valid. This constraint should
/// include collision checks. It does not need to include joint limits or
/// boundary checks. These will be added by the function.
/// \param _rng A random number generator to be used by state samplers
::ompl::base::SpaceInformationPtr createSpaceInformation(
    statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    constraint::TestablePtr _validityConstraint,
    std::unique_ptr<util::RNG> _rng);
}
}
}
#endif
