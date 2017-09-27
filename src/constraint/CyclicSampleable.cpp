#include <aikido/constraint/CyclicSampleable.hpp>

#include <dart/common/StlHelpers.hpp>

namespace aikido {
namespace constraint {

// For internal use only.
class FiniteCyclicSampleGenerator : public SampleGenerator
{
public:
  FiniteCyclicSampleGenerator(std::unique_ptr<SampleGenerator> _generator);

  FiniteCyclicSampleGenerator(const FiniteCyclicSampleGenerator&) = delete;
  FiniteCyclicSampleGenerator(FiniteCyclicSampleGenerator&& other) = delete;

  FiniteCyclicSampleGenerator& operator=(
      const FiniteCyclicSampleGenerator& other)
      = delete;
  FiniteCyclicSampleGenerator& operator=(FiniteCyclicSampleGenerator&& other)
      = delete;

  virtual ~FiniteCyclicSampleGenerator();

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  bool sample(statespace::StateSpace::State* _state) override;

  // Documentation inherited.
  int getNumSamples() const override;

  // Documentation inherited.
  bool canSample() const override;

private:
  statespace::StateSpacePtr mStateSpace;
  std::vector<statespace::StateSpace::State*> mStates;
  std::unique_ptr<SampleGenerator> mGenerator;
  int mIndex;
  int mNumSamples;

  friend class CyclicSampleable;
};

//==============================================================================
FiniteCyclicSampleGenerator::FiniteCyclicSampleGenerator(
    std::unique_ptr<SampleGenerator> _generator)
  : mGenerator(std::move(_generator)), mIndex(0)
{
  if (!mGenerator)
    throw std::invalid_argument("SampleGenerator is nullptr.");

  mNumSamples = mGenerator->getNumSamples();

  if (mNumSamples == SampleGenerator::NO_LIMIT)
    throw std::invalid_argument("SampleGenerator is not finite.");

  if (mNumSamples == 0)
    throw std::invalid_argument("SampleGenerator has 0 samples.");

  mStates.reserve(mNumSamples);

  mStateSpace = mGenerator->getStateSpace();
}

//==============================================================================
FiniteCyclicSampleGenerator::~FiniteCyclicSampleGenerator()
{
  auto space = mGenerator->getStateSpace();

  for (auto state : mStates)
    space->freeState(state);
}

//==============================================================================
statespace::StateSpacePtr FiniteCyclicSampleGenerator::getStateSpace() const
{
  return mGenerator->getStateSpace();
}

//==============================================================================
bool FiniteCyclicSampleGenerator::sample(statespace::StateSpace::State* _state)
{
  if (mGenerator->canSample())
  {
    // Generate a sample.
    bool success = mGenerator->sample(_state);

    if (!success || !_state)
      return false;

    // Copy the sample into mStates.
    auto space = mGenerator->getStateSpace();
    statespace::StateSpace::State* state = space->allocateState();

    space->copyState(_state, state);
    mStates.emplace_back(state);

    return true;
  }

  // Return the sample at mIndex.
  auto space = mGenerator->getStateSpace();
  space->copyState(mStates[mIndex], _state);

  // Set mIndex to 0 if it copied the last element of mStates.
  ++mIndex;
  if (mIndex == mNumSamples)
    mIndex = 0;

  return true;
}

//==============================================================================
int FiniteCyclicSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//==============================================================================
bool FiniteCyclicSampleGenerator::canSample() const
{
  return true;
}

//==============================================================================
CyclicSampleable::CyclicSampleable(SampleablePtr _sampleable)
  : mSampleable(std::move(_sampleable))
{
  if (!mSampleable)
    throw std::invalid_argument("Sampleable is nullptr.");

  int numSamples = mSampleable->createSampleGenerator()->getNumSamples();

  if (numSamples == SampleGenerator::NO_LIMIT)
    throw std::invalid_argument("Sampleable is not finite.");

  if (numSamples == 0)
  {
    throw std::invalid_argument(
        "Sampleable's SampleGenerator produces 0 sample.");
  }

  mStateSpace = mSampleable->getStateSpace();
}

//==============================================================================
statespace::StateSpacePtr CyclicSampleable::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::unique_ptr<SampleGenerator> CyclicSampleable::createSampleGenerator() const
{
  return dart::common::make_unique<FiniteCyclicSampleGenerator>(
      mSampleable->createSampleGenerator());
}

} // namespace constraint
} // namespace aikido
