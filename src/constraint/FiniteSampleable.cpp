#include <iostream>
#include <dart/common/StlHelpers.hpp>
#include <aikido/constraint/FiniteSampleable.hpp>

namespace aikido {
namespace constraint {

// For internal use only.
class FiniteSampleGenerator : public SampleGenerator
{
public:
  // For internal use only.
  FiniteSampleGenerator(
      statespace::ConstStateSpacePtr _stateSpace,
      const std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::
                            ScopedState>& _states);

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
  std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState>
      mStates;
  int mIndex;

  friend class FiniteSampleable;
};

//==============================================================================
FiniteSampleGenerator::FiniteSampleGenerator(
    statespace::ConstStateSpacePtr _stateSpace,
    const std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::
                          ScopedState>& _states)
  : mStateSpace(std::move(_stateSpace)), mIndex(0)
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpacePtr is nullptr.");

  if (_states.empty())
    throw std::invalid_argument("_states is empty.");

  mStates.reserve(_states.size());

  for (const auto& state : _states)
  {
    mStates.emplace_back(state.clone());
  }
}

//==============================================================================
FiniteSampleGenerator::~FiniteSampleGenerator()
{
  // for (auto state : mStates)
  //   mStateSpace->freeState(state);
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
    const aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState& _state)
  : mStateSpace(std::move(_stateSpace))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpacePtr is nullptr.");

  if (!_state)
    throw std::invalid_argument("State is nullptr.");
  mStates.emplace_back(_state.clone());
}

//==============================================================================
FiniteSampleable::FiniteSampleable(
    statespace::StateSpacePtr _stateSpace,
    const std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::
                          ScopedState>& _states)
  : mStateSpace(std::move(_stateSpace))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpacePtr is nullptr.");

  if (_states.empty())
    throw std::invalid_argument("_states is empty.");

  mStates.reserve(_states.size());

  for (const auto& state : _states)
  {
    mStates.emplace_back(state.clone());
  }
}

//==============================================================================
FiniteSampleable::~FiniteSampleable()
{
  // for (const auto state : mStates)
  // {
  //   mStateSpace->freeState(state);
  // }
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
