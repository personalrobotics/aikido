#include <aikido/constraint/FiniteSampleConstraint.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
 FiniteCyclicSampleGenerator::FiniteCyclicSampleGenerator(
 	statespace::StateSpacePtr _stateSpace,
  std::vector<statespace::StateSpace::State*> _states)
: mStateSpace(_stateSpace)
, mStates(_states)
, mIndex(0)
{
  assert(mStateSpace);
  for(auto state: mStates)
  {
  	assert(state);
  }
}


//=============================================================================
statespace::StateSpacePtr FiniteCyclicSampleGenerator::getStateSpace() const
{
	return mStateSpace;
}

//=============================================================================
bool FiniteCyclicSampleGenerator::sample(statespace::StateSpace::State* _state)
{
	if (mStates.size() <= mIndex)
		mIndex = 0;

	mStateSpace->copyState(_state, mStates[mIndex]);
	++mIndex;

	return true;
}

//=============================================================================
int FiniteCyclicSampleGenerator::getNumSamples() const
{
	return NO_LIMIT;
}

//=============================================================================
bool FiniteCyclicSampleGenerator::canSample() const
{
	return true;
}

}
}
