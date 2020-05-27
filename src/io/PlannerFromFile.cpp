#include "aikido/io/PlannerFromFile.hpp"

#include "aikido/io/yaml.hpp"
#include "aikido/planner/SequenceMetaPlanner.hpp"
#include "aikido/planner/SnapConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/vectorfield/VectorFieldPlanner.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"

namespace aikido {
namespace io {
namespace {
using aikido::planner::SnapConfigurationToConfigurationPlanner;
using aikido::planner::ompl::OMPLConfigurationToConfigurationPlanner;
using aikido::statespace::ConstStateSpacePtr;
using aikido::statespace::GeodesicInterpolator;

aikido::planner::PlannerPtr createPlannerFromFile(
    const YAML::Node& plannerNode, const ConstStateSpacePtr stateSpace)
{
  // Snap Planner.
  if (plannerNode["type"].as<std::string>() == "Snap")
  {
    return std::make_shared<SnapConfigurationToConfigurationPlanner>(
        stateSpace, std::make_shared<GeodesicInterpolator>(stateSpace));
  }

  // OMPL Planners.
  if (plannerNode["type"].as<std::string>() == "RRTConnect")
  {
    if (plannerNode["range"])
    {
      // Set the given parameter.
    }
    else
    {
      // Set the default value.
    }
  }
  // If the control has reached here, it is a planner type that is not natively
  // supported for loading. Throw an exception.
  throw std::invalid_argument("Planner type not supported.");
}
} // namespace

std::vector<aikido::planner::PlannerPtr> plannerFromFile(
    aikido::statespace::ConstStateSpacePtr stateSpace,
    const std::string& configurationFile)
{
  // Load the configuration file.
  YAML::Node plannerConfiguration = YAML::LoadFile(configurationFile);

  // Iterate through the planners and generate the appropriate planner types.
  std::vector<aikido::planner::PlannerPtr> planners;
  for (const auto& planner : plannerConfiguration)
  {
    planners.emplace_back(createPlannerFromFile(planner, stateSpace));
  }
  return planners;
}
} // namespace io
} // namespace aikido
