#include "aikido/constraint/SequentialSampleable.hpp"

#include <dart/common/Console.hpp>
#include <dart/common/Memory.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
class SequentialSampleGenerator : public SampleGenerator
{
public:
  /// Sample Generator for Sequential Sampleable
  /// \param[in] stateSpace Statespace the associated constraint operates in
  /// \param[in] generators Sequence of generators associated with corresponding
  /// sequence of sampleables
  SequentialSampleGenerator(
      statespace::StateSpacePtr stateSpace,
      std::vector<std::unique_ptr<SampleGenerator>> generators);

  // Documentation inherited
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited
  bool sample(statespace::StateSpace::State* state) override;

  // Documentation inherited
  int getNumSamples() const override;

  // Documentation inherited
  bool canSample() const override;

private:
  /// StateSpace the associated sampleable operates in.
  statespace::StateSpacePtr mStateSpace;

  /// Sequence of generators associated with corresponding sequence of
  /// sampleables.
  std::vector<std::unique_ptr<SampleGenerator>> mGenerators;

  /// Index of the active sampleable/generator.
  std::size_t mIndex;
};

//==============================================================================
SequentialSampleGenerator::SequentialSampleGenerator(
    statespace::StateSpacePtr stateSpace,
    std::vector<std::unique_ptr<SampleGenerator>> generators)
  : mStateSpace(std::move(stateSpace))
  , mGenerators(std::move(generators))
  , mIndex(0)
{
  for (std::size_t i = 0; i < mGenerators.size(); ++i)
  {
    if (!mGenerators[i])
    {
      std::stringstream msg;
      msg << "Generator " << i << " is nullptr.";
      throw std::invalid_argument(msg.str());
    }
  }

#ifndef NDEBUG
  for (const auto& generator : mGenerators)
    assert(generator->getStateSpace() == mStateSpace);
#endif

  for (std::size_t i = 0u; i < mGenerators.size() - 1; ++i)
  {
    if (mGenerators[i]->getNumSamples() == NO_LIMIT)
      dtwarn << "Sampleable " << i << " is infinite. "
             << mGenerators.size() - i - 1
             << " remaining sampleables will potentially be ignored.";
  }
}

//==============================================================================
statespace::StateSpacePtr SequentialSampleGenerator::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
bool SequentialSampleGenerator::sample(statespace::StateSpace::State* state)
{
  while (mIndex < mGenerators.size())
  {
    if (mGenerators[mIndex]->sample(state))
      return true;

    mIndex++;
  }
  return false;
}

//==============================================================================
int SequentialSampleGenerator::getNumSamples() const
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

//==============================================================================
bool SequentialSampleGenerator::canSample() const
{
  for (std::size_t i = mIndex; i < mGenerators.size(); ++i)
  {
    if (mGenerators[i]->canSample())
      return true;
  }
  return false;
}

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
