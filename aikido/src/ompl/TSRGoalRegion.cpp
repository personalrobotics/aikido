#include <aikido/ompl/TSRGoalRegion.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>

namespace aikido
{
namespace ompl
{
/// Constructor. Generate a goal region from a TSR
TSRGoalRegion::TSRGoalRegion(const ::ompl::base::SpaceInformationPtr& _si,
                             std::unique_ptr<constraint::TSR> _tsr)
    : ::ompl::base::GoalSampleableRegion(_si)
    , mTSR(std::move(_tsr))
    , mSampleGenerator(mTSR->createSampleGenerator())
{
}

/// Sample a state in the goal region
void TSRGoalRegion::sampleGoal(::ompl::base::State* _state) const
{
  auto state = static_cast<AIKIDOGeometricStateSpace::StateType*>(_state);
  mSampleGenerator->sample(state->mState);
}

/// Return the maximum number of samples that can
///  be asked for before repeating
unsigned int TSRGoalRegion::maxSampleCount() const
{
  return mSampleGenerator->getNumSamples();
}

/// Return true of maxSampleCount() > 0
bool TSRGoalRegion::canSample() const { return mSampleGenerator->canSample(); }

/// Compute the distance to the goal (heuristic)
double TSRGoalRegion::distanceGoal(const ::ompl::base::State* _state) const
{
  auto state = static_cast<const AIKIDOGeometricStateSpace::StateType*>(_state);
  Eigen::VectorXd value = mTSR->getValue(state->mState);
  return value.norm();
}

/// Return true if the state satisfies the goal constraints
bool TSRGoalRegion::isSatisfied(const ::ompl::base::State* _state) const
{
  auto state = static_cast<const AIKIDOGeometricStateSpace::StateType*>(_state);
  return mTSR->isSatisfied(state->mState);
}
}
}
