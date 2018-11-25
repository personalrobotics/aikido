#include "aikido/planner/dart/ConcreteSequenceMetaPlanner.hpp"

#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>

namespace aikido {
namespace planner {

using aikido::planner::SnapConfigurationToConfigurationPlanner;
using aikido::statespace::GeodesicInterpolator;

//==============================================================================
ConcreteSequenceMetaPlanner::ConcreteSequenceMetaPlanner(
    statespace::ConstStateSpacePtr stateSpace)
  : SequenceMetaPlanner(std::move(stateSpace), std::vector<PlannerPtr>())
{
  // NOTE: This is the default suite taken frob libherb. This will be updated.
  PlannerPtr snapConfigToConfigPlanner
      = std::make_shared<SnapConfigurationToConfigurationPlanner>(
          stateSpace, std::make_shared<GeodesicInterpolator>(stateSpace));
  mPlanners.push_back(snapConfigToConfigPlanner);

  // TODO.
}

} // namespace planner
} // namespace aikido
