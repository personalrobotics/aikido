#include <aikido/constraint/FiniteSampleConstraint.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
FiniteSampleConstraint::FiniteSampleConstraint(
    statespace::StateSpacePtr _stateSpace,
    statespace::StateSpace::State* _state)
: mStateSpace(_stateSpace)
{
	assert(mStateSpace);

	statespace::StateSpace::State* state = mStateSpace->allocateState();
  mStateSpace->copyState(state, _state);

  mStates.push_back(state);
}


//=============================================================================
FiniteSampleConstraint::FiniteSampleConstraint(
    statespace::StateSpacePtr _stateSpace,
    std::vector<statespace::StateSpace::State*> _states)
: mStateSpace(_stateSpace)
{
	assert(mStateSpace);
	mStates.reserve(_states.size());

	for (auto state: _states)
	{
		statespace::StateSpace::State* newState = mStateSpace->allocateState();
	  mStateSpace->copyState(newState, state);

	  mStates.emplace_back(newState);
	}

}




//=============================================================================
FiniteSampleConstraint::~FiniteSampleConstraint()
{
	for (auto state: mStates)
	{
	  mStateSpace->freeState(state);
	}
}


//=============================================================================
statespace::StateSpacePtr FiniteSampleConstraint::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
std::unique_ptr<SampleGenerator> FiniteSampleConstraint::createSampleGenerator() const
{
  return std::unique_ptr<FiniteSampleGenerator>(new FiniteSampleGenerator(
    mStateSpace,
    mStates));
}


//=============================================================================
std::unique_ptr<SampleGenerator> FiniteSampleConstraint::createCyclicSampleGenerator() const
{
  return std::unique_ptr<FiniteCyclicSampleGenerator>(
  	new FiniteCyclicSampleGenerator(
    mStateSpace,
    mStates));
}

}
}
