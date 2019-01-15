#include <aikido/constraint/FiniteSampleable.hpp>

#include <dart/common/StlHelpers.hpp>

namespace aikido {
namespace constraint {

// For internal use only.
class FiniteSampleGenerator : public SampleGenerator
{
public:
  // For internal use only.
  FiniteSampleGenerator(
      statespace::ConstStateSpacePtr _stateSpace,
      const std::vector<statespace::StateSpace::State*>& _states);

  FiniteSampleGenerator(const FiniteSampleGenerator&) = delete;
  FiniteSampleGenerator(FiniteSampleGenerator&& other) = delete;

  FiniteSampleGenerator& operator=(const FiniteSampleGenerator& other) = delete;
  FiniteSampleGenerator& operator=(FiniteSampleGenerator&& other) = delete;

  virtual ~FiniteSampleGenerator();

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  bool sample(statespace::StateSpace::State* _state) override;

  // Documentation inherited.
  int getNumSamples() const override;

  // Documentation inherited.
  bool canSample() const override;

private:
  statespace::ConstStateSpacePtr mStateSpace;
  std::vector<statespace::StateSpace::State*> mStates;
  int mIndex;

  friend class FiniteSampleable;
};

//==============================================================================
FiniteSampleGenerator::FiniteSampleGenerator(
    statespace::ConstStateSpacePtr _stateSpace,
    const std::vector<statespace::StateSpace::State*>& _states)
  : mStateSpace(std::move(_stateSpace)), mIndex(0)
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpacePtr is nullptr.");

  if (_states.empty())
    throw std::invalid_argument("_states is empty.");

  mStates.reserve(_states.size());

  for (auto state : _states)
  {
    if (!state)
    {
      // Free all states already saved in mStates.
      for (auto saved : mStates)
        mStateSpace->freeState(saved);

      throw std::invalid_argument("One of the states in _states is nullptr.");
    }

    statespace::StateSpace::State* newState = mStateSpace->allocateState();
    mStateSpace->copyState(state, newState);

    mStates.emplace_back(newState);
  }
}

//==============================================================================
FiniteSampleGenerator::~FiniteSampleGenerator()
{
  for (auto state : mStates)
    mStateSpace->freeState(state);
}

//==============================================================================
statespace::ConstStateSpacePtr FiniteSampleGenerator::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
bool FiniteSampleGenerator::sample(statespace::StateSpace::State* _state)
{
  if (mStates.size() <= static_cast<std::size_t>(mIndex))
    return false;

  mStateSpace->copyState(mStates[mIndex], _state);
  ++mIndex;

  return true;
}

//==============================================================================
int FiniteSampleGenerator::getNumSamples() const
{
  return mStates.size() - mIndex;
}

//==============================================================================
bool FiniteSampleGenerator::canSample() const
{
  return mStates.size() > static_cast<std::size_t>(mIndex);
}

//==============================================================================
FiniteSampleable::FiniteSampleable(
    statespace::StateSpacePtr _stateSpace,
    const statespace::StateSpace::State* _state)
  : mStateSpace(std::move(_stateSpace))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpacePtr is nullptr.");

  if (!_state)
    throw std::invalid_argument("State is nullptr.");

  statespace::StateSpace::State* state = mStateSpace->allocateState();
  mStateSpace->copyState(_state, state);

  mStates.push_back(state);
}

//==============================================================================
FiniteSampleable::FiniteSampleable(
    statespace::StateSpacePtr _stateSpace,
    const std::vector<const statespace::StateSpace::State*>& _states)
  : mStateSpace(std::move(_stateSpace))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpacePtr is nullptr.");

  if (_states.empty())
    throw std::invalid_argument("_states is empty.");

  mStates.reserve(_states.size());

  for (auto state : _states)
  {
    if (!state)
    {
      // Free all states already saved in mStates.
      for (auto saved : mStates)
        mStateSpace->freeState(saved);

      throw std::invalid_argument("One of the states in _states is nullptr.");
    }

    statespace::StateSpace::State* newState = mStateSpace->allocateState();
    mStateSpace->copyState(state, newState);

    mStates.emplace_back(newState);
  }
}

//==============================================================================
FiniteSampleable::~FiniteSampleable()
{
  for (auto state : mStates)
  {
    mStateSpace->freeState(state);
  }
}

//==============================================================================
statespace::ConstStateSpacePtr FiniteSampleable::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::unique_ptr<SampleGenerator> FiniteSampleable::createSampleGenerator() const
{
  return ::dart::common::make_unique<FiniteSampleGenerator>(
      mStateSpace, mStates);
}

} // namespace constraint
} // namespace aikido
