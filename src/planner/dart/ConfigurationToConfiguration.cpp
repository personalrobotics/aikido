#include "aikido/planner/dart/ConfigurationToConfiguration.hpp"
#include "aikido/constraint/dart/DartConstraint.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfiguration::ConfigurationToConfiguration(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
    const statespace::dart::MetaSkeletonStateSpace::State* goalState,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(mMetaSkeletonStateSpace->createState())
  , mGoalState(stateSpace->cloneState(goalState))
{
  // Do nothing.
}

//==============================================================================
ConfigurationToConfiguration::ConfigurationToConfiguration(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
    const statespace::dart::MetaSkeletonStateSpace::State* goalState,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(nullptr)
  , mStartState(stateSpace->cloneState(startState))
  , mGoalState(stateSpace->cloneState(goalState))
{
  // Do nothing.
}

//==============================================================================
const std::string& ConfigurationToConfiguration::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToConfiguration::getStaticType()
{
  static std::string name("ConfigurationToConfiguration");
  return name;
}

//==============================================================================
const statespace::dart::MetaSkeletonStateSpace::State*
ConfigurationToConfiguration::getStartState() const
{
  // Take start state from MetaSkeleton if passed. Store in the ScopedState
  // instance variable to avoid dangling pointers.
  if (mMetaSkeleton)
    mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), mStartState);

  return mStartState;
}

//==============================================================================
const statespace::dart::MetaSkeletonStateSpace::State*
ConfigurationToConfiguration::getGoalState() const
{
  return mGoalState;
}

//==============================================================================
std::shared_ptr<Problem> ConfigurationToConfiguration::clone() const
{
  throw std::runtime_error(
      "dart::ConfigurationToConfiguration should call clone with metaSkeleton.");
  /*
  // TODO:assert that metaSkeleton is consistent with mMetaSkeletonStateSpace
  auto problem = std::make_shared<ConfigurationToConfiguration>
    (mMetaSkeletonStateSpace, mStartState,
     mGoalState, mConstraint);

  problem->mStartState = mStartState.clone();

  return problem;
  */
}

//==============================================================================
std::shared_ptr<Problem> ConfigurationToConfiguration::clone(
      ::dart::collision::CollisionDetectorPtr collisionDetector,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton) const
{
  using aikido::constraint::dart::DartConstraint;
  auto constraint = std::dynamic_pointer_cast<const DartConstraint>(mConstraint);

  if (constraint)
  {
    // TODO:assert that metaSkeleton is consistent with mMetaSkeletonStateSpace
    auto problem = std::make_shared<ConfigurationToConfiguration>
      (mMetaSkeletonStateSpace, mStartState,
       mGoalState, constraint->clone(collisionDetector, metaSkeleton));

    // GL: shall we do this for all cloned methods?
    problem->mStartState = mStartState.clone();

    return problem;
  }else
  {
    auto problem = std::make_shared<ConfigurationToConfiguration>
      (mMetaSkeletonStateSpace, mStartState,
       mGoalState, mConstraint);

    problem->mStartState = mStartState.clone();

    return problem;
  }

}


} // namespace dart
} // namespace planner
} // namespace aikido
