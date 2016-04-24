#ifndef AIKIDO_OMPL_DART_H_
#define AIKIDO_OMPL_DART_H_

#include <dart/dynamics/Skeleton.h>
#include <ompl/base/SpaceInformation.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

namespace aikido
{
namespace ompl
{
/// Create a SpaceInformation by constructing default StateSpace and metrics for
/// the given robot
::ompl::base::SpaceInformationPtr createSpaceInformation(
    const statespace::MetaSkeletonStateSpacePtr &_stateSpace,
    std::unique_ptr<util::RNG> _rng);
}
}

#endif
