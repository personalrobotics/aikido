#include "aikido/planner/dart/ConcreteSequenceMetaPlanner.hpp"

#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToTSR.hpp>
#include <aikido/planner/vectorfield/VectorFieldConfigurationToEndEffectorOffsetPlanner.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>

namespace aikido {
namespace planner {

using aikido::planner::SnapConfigurationToConfigurationPlanner;
using aikido::planner::dart::ConfigurationToConfiguration_to_ConfigurationToTSR;
using aikido::planner::vectorfield::
    VectorFieldConfigurationToEndEffectorOffsetPlanner;
using aikido::statespace::GeodesicInterpolator;

//==============================================================================
ConcreteSequenceMetaPlanner::ConcreteSequenceMetaPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : SequenceMetaPlanner(std::move(stateSpace), std::vector<PlannerPtr>())
{
  // NOTE: This is the default suite taken from libherb. This will be updated.
  auto snapConfigToConfigPlanner
      = std::make_shared<SnapConfigurationToConfigurationPlanner>(
          stateSpace, std::make_shared<GeodesicInterpolator>(stateSpace));
  mPlanners.push_back(snapConfigToConfigPlanner);

  auto snapConfigToTSRPlanner
      = std::make_shared<ConfigurationToConfiguration_to_ConfigurationToTSR>(
          snapConfigToConfigPlanner, metaSkeleton);
  mPlanners.push_back(snapConfigToTSRPlanner);

  // TODO: (sniyaz) Where should these params be stored?
  double distanceTolerance = 1e-2;
  double positionTolerance = 0.01;
  double angularTolerance = 0.01;
  double initialStepSize = 0.005;
  double jointLimitTolerance = 1e-3;
  double constraintCheckResolution = 1e-2;
  auto planningTimeout = std::chrono::duration<double>(5.0);
  auto vfpOffsetPlanner
      = std::make_shared<VectorFieldConfigurationToEndEffectorOffsetPlanner>(
          stateSpace,
          metaSkeleton,
          distanceTolerance,
          positionTolerance,
          angularTolerance,
          initialStepSize,
          jointLimitTolerance,
          constraintCheckResolution,
          planningTimeout);
}

} // namespace planner
} // namespace aikido
