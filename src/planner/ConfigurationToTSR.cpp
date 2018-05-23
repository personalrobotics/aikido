#include "aikido/planner/ConfigurationToTSR.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToTSR::ConfigurationToTSR(
    statespace::ConstStateSpacePtr stateSpace,
    dart::dynamics::MetaSkeletonPtr metaSkeleton,
    dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    std::size_t maxNumTrials,
    const statespace::StateSpace::State* startState,
    constraint::dart::ConstTSRPtr goalTSR,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace), std::move(constraint))
  , mMetaSkeleton(std::move(metaSkeleton))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mMaxNumTrials(maxNumTrials)
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
dart::dynamics::MetaSkeletonPtr ConfigurationToTSR::getMetaSkeleton()
{
  return mMetaSkeleton;
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr ConfigurationToTSR::getMetaSkeleton() const
{
  return mMetaSkeleton;
}

//==============================================================================
dart::dynamics::ConstBodyNodePtr ConfigurationToTSR::getEndEffectorBodyNode()
    const
{
  return mEndEffectorBodyNode;
}

//==============================================================================
std::size_t ConfigurationToTSR::getMaxNumTrials() const
{
  return mMaxNumTrials;
}

//==============================================================================
const statespace::StateSpace::State* ConfigurationToTSR::getStartState() const
{
  return mStartState;
}

//==============================================================================
constraint::dart::ConstTSRPtr ConfigurationToTSR::getGoalTSR() const
{
  return mGoalTSR;
}

} // namespace planner
} // namespace aikido
