#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/distance/defaults.hpp"
#include "aikido/planner/ompl/Planner.hpp"
#include "aikido/planner/ompl/dart.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace ompl {

::ompl::base::SpaceInformationPtr createSpaceInformation(
    statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    constraint::TestablePtr _validityConstraint,
    double _maxDistanceBtwValidityChecks,
    std::unique_ptr<common::RNG> _rng)
{

  // Distance metric
  auto dmetric = distance::createDistanceMetric(_stateSpace);

  // State sampler
  auto sampler
      = constraint::dart::createSampleableBounds(_stateSpace, std::move(_rng));

  // Bounds constraint
  auto boundsConstraint = constraint::dart::createTestableBounds(_stateSpace);

  // Projectable constraint - project to statespace
  auto boundsProjection
      = constraint::dart::createProjectableBounds(_stateSpace);

  // Interpolator
  auto interpolator
      = std::make_shared<statespace::GeodesicInterpolator>(_stateSpace);

  return getSpaceInformation(
      std::move(_stateSpace),
      std::move(interpolator),
      std::move(dmetric),
      std::move(sampler),
      std::move(_validityConstraint),
      std::move(boundsConstraint),
      std::move(boundsProjection),
      _maxDistanceBtwValidityChecks);
}

} // namespace ompl
} // namespace planner
} // namespace aikido
