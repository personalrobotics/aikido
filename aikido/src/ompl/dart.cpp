#include <aikido/ompl/dart.hpp>
#include <aikido/ompl/OMPLPlanner.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/distance/DistanceMetricDefaults.hpp>
#include <aikido/constraint/dart.hpp>

namespace aikido
{
namespace ompl
{
::ompl::base::SpaceInformationPtr createSpaceInformation(
    const statespace::MetaSkeletonStateSpacePtr &_stateSpace,
    std::unique_ptr<util::RNG> _rng)
{

  // Distance metric
  auto dmetric = distance::createDistanceMetricFor(_stateSpace);

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
                             std::move(boundsConstraint),
                             std::move(boundsProjection));
}
}
}
