#include <aikido/constraint/FiniteCyclicSampleConstraint.hpp>

namespace aikido {
namespace constraint {


// For internal use only.
class FiniteCyclicSampleGenerator : public SampleGenerator
{
public:

  FiniteCyclicSampleGenerator(const FiniteCyclicSampleGenerator&) = delete;
  FiniteCyclicSampleGenerator(FiniteCyclicSampleGenerator&& other) = delete;

  FiniteCyclicSampleGenerator& operator=(
    const FiniteCyclicSampleGenerator& other) = delete;
  FiniteCyclicSampleGenerator& operator=(
    FiniteCyclicSampleGenerator&& other) = delete;

  virtual ~FiniteCyclicSampleGenerator() = default; 

  /// Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Documentation inherited.
  bool sample(statespace::StateSpace::State* _state) override;

  /// Documentation inherited.
  int getNumSamples() const override;

  /// Documentation inherited.
  bool canSample() const override;

private:

  // For internal use only.
  FiniteCyclicSampleGenerator(
    statespace::StateSpacePtr _stateSpace,
    std::vector<statespace::StateSpace::State*> _states);

  statespace::StateSpacePtr mStateSpace;
  std::vector<statespace::StateSpace::State*> mStates;
  int mIndex;

  friend class FiniteCyclicSampleConstraint;
};

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

//=============================================================================
FiniteCyclicSampleConstraint::FiniteCyclicSampleConstraint(
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
FiniteCyclicSampleConstraint::FiniteCyclicSampleConstraint(
  statespace::StateSpacePtr _stateSpace,
  std::vector<const statespace::StateSpace::State*> _states)
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
FiniteCyclicSampleConstraint::~FiniteCyclicSampleConstraint()
{
  for (auto state: mStates)
  {
    mStateSpace->freeState(state);
  }
}

//=============================================================================
statespace::StateSpacePtr FiniteCyclicSampleConstraint::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
std::unique_ptr<SampleGenerator> FiniteCyclicSampleConstraint::createSampleGenerator() const
{
  return std::unique_ptr<FiniteCyclicSampleGenerator>(
    new FiniteCyclicSampleGenerator(
    mStateSpace,
    mStates));
}

}
}
