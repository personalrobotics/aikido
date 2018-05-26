#include "aikido/planner/dart/ConfigurationToTSR.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToTSR::ConfigurationToTSR(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
    std::size_t maxSamples,
    constraint::dart::ConstTSRPtr goalTSR,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace), std::move(constraint))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mMaxSamples(maxSamples)
  , mStartState(startState)
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
const statespace::dart::MetaSkeletonStateSpace::State*
ConfigurationToTSR::getStartState() const
{
  return mStartState;
}

//==============================================================================
constraint::dart::ConstTSRPtr ConfigurationToTSR::getGoalTSR() const
{
  return mGoalTSR;
}

} // namespace dart
} // namespace planner
} // namespace aikido
