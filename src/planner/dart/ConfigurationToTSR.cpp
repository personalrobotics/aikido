#include "aikido/planner/dart/ConfigurationToTSR.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToTSR::ConfigurationToTSR(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    std::size_t maxSamples,
    constraint::dart::ConstTSRPtr goalTSR,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mMaxSamples(maxSamples)
  , mGoalTSR(goalTSR)
{
  // Do nothing
}

//==============================================================================
const std::string& ConfigurationToTSR::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToTSR::getStaticType()
{
  static std::string name("ConfigurationToTSR");
  return name;
}

//==============================================================================
::dart::dynamics::ConstBodyNodePtr ConfigurationToTSR::getEndEffectorBodyNode()
    const
{
  return mEndEffectorBodyNode;
}

//==============================================================================
std::size_t ConfigurationToTSR::getMaxSamples() const
{
  return mMaxSamples;
}

//==============================================================================
constraint::dart::ConstTSRPtr ConfigurationToTSR::getGoalTSR() const
{
  return mGoalTSR;
}

} // namespace dart
} // namespace planner
} // namespace aikido
