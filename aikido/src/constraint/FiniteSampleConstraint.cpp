#include <aikido/constraint/FiniteSampleConstraint.hpp>

namespace aikido {
namespace constraint {

// For internal use only.
class FiniteSampleGenerator : public SampleGenerator
{
public:

  FiniteSampleGenerator(const FiniteSampleGenerator&) = delete;
  FiniteSampleGenerator(FiniteSampleGenerator&& other) = delete;

  FiniteSampleGenerator& operator=(
    const FiniteSampleGenerator& other) = delete;
  FiniteSampleGenerator& operator=(
    FiniteSampleGenerator&& other) = delete;

  virtual ~FiniteSampleGenerator(); 

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
  FiniteSampleGenerator(
    statespace::StateSpacePtr _stateSpace,
    const std::vector<statespace::StateSpace::State*>& _states);

  statespace::StateSpacePtr mStateSpace;
  std::vector<statespace::StateSpace::State*> mStates;
  int mIndex;

  friend class FiniteSampleConstraint;
};

//=============================================================================
 FiniteSampleGenerator::FiniteSampleGenerator(
  statespace::StateSpacePtr _stateSpace,
  const std::vector<statespace::StateSpace::State*>& _states)
: mStateSpace(std::move(_stateSpace))
, mIndex(0)
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpacePtr is nullptr.");

  if (_states.empty())
    throw std::invalid_argument("_states is empty.");

  mStates.reserve(_states.size());

  for (auto state: _states)
  {
    if (!state){
      // Free all states already saved in mStates.
      for (auto saved: mStates)
        mStateSpace->freeState(saved);

      throw std::invalid_argument("One of the states in _states is nullptr.");
    }

    statespace::StateSpace::State* newState = mStateSpace->allocateState();
    mStateSpace->copyState(newState, state);

    mStates.emplace_back(newState);
  }
}

//=============================================================================
FiniteSampleGenerator::~FiniteSampleGenerator()
{
  for (auto state: mStates)
  {
    mStateSpace->freeState(state);
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

//=============================================================================
FiniteSampleConstraint::FiniteSampleConstraint(
  statespace::StateSpacePtr _stateSpace,
  const statespace::StateSpace::State* _state)
: mStateSpace(std::move(_stateSpace))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpacePtr is nullptr.");

  if (!_state)
    throw std::invalid_argument("State is nullptr.");

  statespace::StateSpace::State* state = mStateSpace->allocateState();
  mStateSpace->copyState(state, _state);

  mStates.push_back(state);
}


//=============================================================================
FiniteSampleConstraint::FiniteSampleConstraint(
  statespace::StateSpacePtr _stateSpace,
    const std::vector<const statespace::StateSpace::State*>& _states)
: mStateSpace(std::move(_stateSpace))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpacePtr is nullptr.");

  if (_states.empty())
    throw std::invalid_argument("_states is empty.");

  mStates.reserve(_states.size());

  for (auto state: _states)
  {
    if (!state){
      // Free all states already saved in mStates.
      for (auto saved: mStates)
        mStateSpace->freeState(saved);

      throw std::invalid_argument("One of the states in _states is nullptr.");
    }

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

}
}
