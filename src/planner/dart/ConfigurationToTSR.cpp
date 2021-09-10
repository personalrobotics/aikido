#include "aikido/planner/dart/ConfigurationToTSR.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToTSR::ConfigurationToTSR(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    std::size_t maxSamplingTries,
    std::size_t batchSize,
    std::size_t maxBatches,
    constraint::dart::ConstTSRPtr goalTSR,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(mMetaSkeletonStateSpace->createState())
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mMaxSamplingTries(maxSamplingTries)
  , mBatchSize(batchSize)
  , mMaxBatches(maxBatches)
  , mGoalTSR(goalTSR)
{
  // Do nothing.
}

//==============================================================================
ConfigurationToTSR::ConfigurationToTSR(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    std::size_t maxSamplingTries,
    std::size_t batchSize,
    std::size_t maxBatches,
    constraint::dart::ConstTSRPtr goalTSR,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(nullptr)
  , mStartState(stateSpace->cloneState(startState))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mMaxSamplingTries(maxSamplingTries)
  , mBatchSize(batchSize)
  , mMaxBatches(maxBatches)
  , mGoalTSR(goalTSR)
{
  // Do nothing.
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
std::size_t ConfigurationToTSR::getMaxSamplingTries() const
{
  return mMaxSamplingTries;
}

//==============================================================================
std::size_t ConfigurationToTSR::getBatchSize() const
{
  return mBatchSize;
}

//==============================================================================
std::size_t ConfigurationToTSR::getMaxBatches() const
{
  return mMaxBatches;
}

//==============================================================================
const statespace::dart::MetaSkeletonStateSpace::State*
ConfigurationToTSR::getStartState() const
{
  // Take start state from MetaSkeleton if passed. Store in the ScopedState
  // instance variable to avoid dangling pointers.
  if (mMetaSkeleton)
    mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), mStartState);

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
