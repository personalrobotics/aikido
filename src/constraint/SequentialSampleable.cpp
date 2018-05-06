#include "aikido/constraint/SequentialSampleable.hpp"

#include <dart/common/StlHelpers.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
class SequentialSampleGenerator : public SampleGenerator
{
public:
  SequentialSampleGenerator(
      statespace::StateSpacePtr stateSpace,
      std::vector<std::unique_ptr<SampleGenerator>> generators)
    : mStateSpace(std::move(stateSpace))
    , mGenerators(std::move(generators))
    , mIndex(0)
  {
    #ifndef NDEBUG
      for (const auto& generator : mGenerators)
        assert(generator->getStateSpace() == mStateSpace);
    #endif
  }

  // Documentation inherited
  statespace::StateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  // Documentation inherited
  bool sample(statespace::StateSpace::State* state)
  {
    while (canSample())
    {
      if (mGenerators[mIndex]->canSample())
      {
        if (mGenerators[mIndex]->sample(state))
          return true;
      }
      mIndex++;
    }
    return false;
  }

  // Documentation inherited
  int getNumSamples() const override
  {
    int numSamples = 0;
    for (std::size_t i = mIndex; i < mGenerators.size(); ++i)
    {
      if (mGenerators[i]->getNumSamples() == NO_LIMIT)
        return NO_LIMIT;

      numSamples += mGenerators[i]->getNumSamples();
    }
    return numSamples;
  }

  // Documentation inherited
  bool canSample() const override
  {
    for (std::size_t i = mIndex; i < mGenerators.size(); ++i)
    {
      if (mGenerators[i]->canSample())
        return true;
    }
    return false;
  }

private:
  /// StateSpace the associated sampleable operates in.
  statespace::StateSpacePtr mStateSpace;

  /// Set of generators associated with corresponding set of sampleables.
  std::vector<std::unique_ptr<SampleGenerator>> mGenerators;

  /// Index of the active sampleable/generator.
  std::size_t mIndex;
};

//==============================================================================
SequentialSampleable::SequentialSampleable(
    statespace::StateSpacePtr stateSpace,
    const std::vector<ConstSampleablePtr>& sampleables)
  : mStateSpace(std::move(stateSpace)), mSampleables(sampleables)
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpacePtr is nullptr.");

  if (mSampleables.empty())
    throw std::invalid_argument("Vector of Sampleables is empty.");

  for (std::size_t i = 0; i < mSampleables.size(); ++i)
  {
    auto sampleable = mSampleables[i];
    if (!sampleable)
    {
      std::stringstream msg;
      msg << "Sampleable " << i << " is nullptr.";
      throw std::invalid_argument(msg.str());
    }

    if (sampleable->getStateSpace() != mStateSpace)
    {
      std::stringstream msg;
      msg << "Mismatch between statespace of Sampleable " << i
          << " and given statespace.";
      throw std::invalid_argument(msg.str());
    }
  }
  // TODO (avk): Somewhere we need to give warning if initial samplers are
  // infinite or if all are finite.
}

//==============================================================================
statespace::StateSpacePtr SequentialSampleable::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::unique_ptr<SampleGenerator> SequentialSampleable::createSampleGenerator()
    const
{
  std::vector<std::unique_ptr<SampleGenerator>> generators;
  generators.reserve(mSampleables.size());

  for (const auto& sampleable : mSampleables)
    generators.emplace_back(sampleable->createSampleGenerator());

  return dart::common::make_unique<SequentialSampleGenerator>(
      mStateSpace, std::move(generators));
}

} // namespace constraint
} // namespace aikido
