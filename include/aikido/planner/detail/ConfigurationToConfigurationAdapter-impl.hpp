#include "aikido/planner/PlannerAdapter.hpp"

namespace aikido {
namespace planner {

//==============================================================================
template <>
bool ConfigurationToConfigurationAdapter::canSolve(const Problem& problem) const
{
  if (problem.getType() == ConfigurationToConfiguration::getStaticType())
    return true;
  if (problem.getType() == ConfigurationToTSR::getStaticType())
    return true;
  return false;
}

//==============================================================================
template <>
trajectory::TrajectoryPtr ConfigurationToConfigurationAdapter::plan(
    const ConfigurationToConfiguration& problem, Planner::Result* result)
{
  return mDelegate->plan(problem, result);
}

//==============================================================================
template <>
trajectory::TrajectoryPtr ConfigurationToConfigurationAdapter::plan(
    const ConfigurationToConfigurations& /*problem*/,
    Planner::Result* /*result*/)
{
  // TODO
  return nullptr;
}

//==============================================================================
template <>
trajectory::TrajectoryPtr ConfigurationToConfigurationAdapter::plan(
    const ConfigurationToEndEffectorOffset& /*problem*/,
    Planner::Result* /*result*/)
{
  // TODO
  return nullptr;
}

//==============================================================================
template <>
trajectory::TrajectoryPtr ConfigurationToConfigurationAdapter::plan(
    const ConfigurationToEndEffectorPose& /*problem*/,
    Planner::Result* /*result*/)
{
  // TODO
  return nullptr;
}

//==============================================================================
template <>
trajectory::TrajectoryPtr ConfigurationToConfigurationAdapter::plan(
    const ConfigurationToTSR& /*problem*/, Planner::Result* /*result*/)
{
  // TODO
  return nullptr;
}

//==============================================================================
template <>
trajectory::TrajectoryPtr ConfigurationToConfigurationAdapter::plan(
    const Problem& problem, Planner::Result* result)
{
  if (!canSolve(problem))
    return nullptr;

  if (problem.getType() == ConfigurationToConfiguration::getStaticType())
    return plan(
        static_cast<const ConfigurationToConfiguration&>(problem), result);
  if (problem.getType() == ConfigurationToTSR::getStaticType())
    return plan(static_cast<const ConfigurationToTSR&>(problem), result);

  return nullptr;
}

} // namespace planner
} // namespace aikido
