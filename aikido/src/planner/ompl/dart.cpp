#include <aikido/planner/ompl/dart.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/constraint/dart.hpp>

namespace aikido {
namespace planner {
namespace ompl {
::ompl::base::SpaceInformationPtr createSpaceInformation(
    statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    constraint::TestablePtr _validityConstraint,
    std::unique_ptr<util::RNG> _rng)
{

  // Distance metric
  auto dmetric = distance::createDistanceMetric(_stateSpace);

  // State sampler
  auto sampler = constraint::createSampleableBounds(_stateSpace, std::move(_rng));

  // Bounds constraint
  auto boundsConstraint = constraint::createTestableBounds(_stateSpace);

  // Projectable constraint - project to statespace
  auto boundsProjection = constraint::createProjectableBounds(_stateSpace);

  // Interpolator
  auto interpolator =
      std::make_shared<statespace::GeodesicInterpolator>(_stateSpace);

  return getSpaceInformation(std::move(_stateSpace), std::move(interpolator),
                             std::move(dmetric), std::move(sampler),
                             std::move(_validityConstraint),
                             std::move(boundsConstraint),
                             std::move(boundsProjection));
}
}
}
}
