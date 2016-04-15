#include <aikido/ompl/GoalRegion.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>

namespace aikido
{
namespace ompl
{
/// Constructor. Generate a goal region from a TSR
GoalRegion::GoalRegion(const ::ompl::base::SpaceInformationPtr& _si,
                       const constraint::TestableConstraintPtr& _goalTestable,
                       std::unique_ptr<constraint::SampleGenerator> _generator)
    : ::ompl::base::GoalSampleableRegion(_si)
    , mSampleGenerator(std::move(_generator))
    , mTestable(std::move(_goalTestable))
{
}

/// Sample a state in the goal region
void GoalRegion::sampleGoal(::ompl::base::State* _state) const
{
  auto state = static_cast<AIKIDOGeometricStateSpace::StateType*>(_state);
  bool valid = mSampleGenerator->sample(state->mState);
  if(!valid){
      throw std::runtime_error("Failed to sample a valid goal");
  }
  
}

/// Return the maximum number of samples that can
///  be asked for before repeating
unsigned int GoalRegion::maxSampleCount() const
{
  return mSampleGenerator->getNumSamples();
}

/// Return true of maxSampleCount() > 0
bool GoalRegion::canSample() const { return mSampleGenerator->canSample(); }

/// Compute the distance to the goal (heuristic)
double GoalRegion::distanceGoal(const ::ompl::base::State* _state) const
{
  if (isSatisfied(_state)) {
    return 0;
  }
  return std::numeric_limits<double>::infinity();
}

/// Return true if the state satisfies the goal constraints
bool GoalRegion::isSatisfied(const ::ompl::base::State* _state) const
{
  auto state = static_cast<const AIKIDOGeometricStateSpace::StateType*>(_state);
  return mTestable->isSatisfied(state->mState);
}
}
}
