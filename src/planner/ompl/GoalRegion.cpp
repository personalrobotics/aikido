#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/planner/ompl/GoalRegion.hpp>

namespace aikido {
namespace planner {
namespace ompl {

//==============================================================================
GoalRegion::GoalRegion(
    const ::ompl::base::SpaceInformationPtr& _si,
    constraint::TestablePtr _goalTestable,
    std::unique_ptr<constraint::SampleGenerator> _generator)
  : ::ompl::base::GoalSampleableRegion(_si)
  , mTestable(std::move(_goalTestable))
  , mSampleGenerator(std::move(_generator))
{
  if (_si == nullptr)
  {
    throw std::invalid_argument("SpaceInformation is null");
  }

  if (mTestable == nullptr)
  {
    throw std::invalid_argument("Testable is null");
  }

  if (mSampleGenerator == nullptr)
  {
    throw std::invalid_argument("SampleGenerator is null");
  }

  if (mSampleGenerator->getStateSpace() != mTestable->getStateSpace())
  {
    throw std::invalid_argument(
        "SampleGenerator and Testable defined on different statespaces.");
  }
}

//==============================================================================
void GoalRegion::sampleGoal(::ompl::base::State* _state) const
{
  auto state = static_cast<GeometricStateSpace::StateType*>(_state);
  bool valid = false;
  if (mSampleGenerator->canSample())
  {
    valid = mSampleGenerator->sample(state->mState);
  }
  state->mValid = valid;
}

//==============================================================================
unsigned int GoalRegion::maxSampleCount() const
{
  return mSampleGenerator->getNumSamples();
}

//==============================================================================
bool GoalRegion::couldSample() const
{
  return mSampleGenerator->canSample();
}

//==============================================================================
double GoalRegion::distanceGoal(const ::ompl::base::State* _state) const
{
  if (isSatisfied(_state))
  {
    return 0;
  }
  return std::numeric_limits<double>::infinity();
}

//==============================================================================
bool GoalRegion::isSatisfied(const ::ompl::base::State* _state) const
{
  auto state = static_cast<const GeometricStateSpace::StateType*>(_state);
  if (state == nullptr || state->mState == nullptr)
    return false;
  return mTestable->isSatisfied(state->mState);
}
} // namespace ompl
} // namespace planner
} // namespace aikido
