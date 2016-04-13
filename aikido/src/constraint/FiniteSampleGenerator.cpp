#include <aikido/constraint/FiniteSampleConstraint.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
 FiniteSampleGenerator::FiniteSampleGenerator(
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
statespace::StateSpacePtr FiniteSampleGenerator::getStateSpace() const
{
	return mStateSpace;
}

//=============================================================================
bool FiniteSampleGenerator::sample(statespace::StateSpace::State* _state)
{
	if (mStates.size() <= mIndex)
		return false;

	mStateSpace->copyState(_state, mStates[mIndex]);
	++mIndex;

	return true;
}

//=============================================================================
int FiniteSampleGenerator::getNumSamples() const
{
	return mStates.size() - mIndex;
}

//=============================================================================
bool FiniteSampleGenerator::canSample() const
{
	return mStates.size() > mIndex;
}

}
}
