#include <map>
#include "aikido/io/Planner.hpp"

#include "aikido/io/detail/yaml_extension.hpp"
#include "aikido/io/yaml.hpp"

using aikido::statespace::ConstStateSpacePtr;

namespace aikido {
namespace io {

//==============================================================================
aikido::planner::PlannerPtr constructPlanner(ConstStateSpacePtr space,
                                             const std::string& configurationFile)
{
  if (!space)
    throw std::invalid_argument("Planner statespace cannot be a nullptr.");
  // TODO (avk): Check if the file given exists, otherwise die gracefully.

  YAML::Node configuration = YAML::LoadFile(configurationFile);
  const YAML::Node& planners = configuration["planners"];

  // Iterate through the planners and construct them accordingly.
  std::map<std::string, std::string> currentPlannerConfiguration;
  for (YAML::const_iterator it = planners.begin() ;it != planners.end(); ++it)
  {
    currentPlannerConfiguration.clear();
    std::string plannerName = it->first.as<std::string>();
    const YAML::Node& planners = configuration[plannerName];

    // Construct the map of planners parameters.
    for (YAML::const_iterator it = planners.begin() ;it != planners.end(); ++it)

    if ()
    aikido::planner::ompl::OMPLConfigurationToConfigurationPlanner
     std::string key = it->first.as<std::string>();       // <- key
     cTypeList.push_back(it->second.as<CharacterType>()); // <- value
  }


  if ()
  return nullptr;
}

} // namespace io
} // namespace aikido
