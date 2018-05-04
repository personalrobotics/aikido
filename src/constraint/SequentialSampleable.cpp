#include <aikido/constraint/SequentialSampleable.hpp>

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
    : mStateSpace(std::move(stateSpace)), mGenerators(std::move(generators))
  {
    // Do nothing
  }

  statespace::StateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  bool sample(statespace::StateSpace::State* state) override
  {
    return true;
  }

  int getNumSamples() const override
  {
    return 0;
  }

  bool canSample() const override
  {
    return false;
  }

private:
  statespace::StateSpacePtr mStateSpace;
  std::vector<std::unique_ptr<SampleGenerator>> mGenerators;
};

//==============================================================================
SequentialSampleable::SequentialSampleable(statespace::StateSpacePtr stateSpace,
                                           std::vector<SampleablePtr> sampleables)
  : mStateSpace(std::move(stateSpace)), mSampleables(std::move(sampleables))
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
      msg << "Sampleable " << i << "is nullptr.";
      throw std::invalid_argument(msg.str());
    }

    if (sampleable->getStateSpace != mStateSpace)
    {
      std::stringstream msg;
      msg << "Mismatch between statespace of Sampleable " << i << "and given statespace.";
      throw std::invalid_argument(msg.str());
    }
  }
  // TODO (avk): Somewhere we need to give warning if initial samplers are infinite
  // or if all are finite.
}

//==============================================================================
statespace::StateSpacePtr SequentialSampleable::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::unique_ptr<SampleGenerator> SequentialSampleable::createSampleGenerator() const
{
  std::vector<std::unique_ptr<SampleGenerator>> generators;
  generators.reserve(mSampleables.size());

  for (const auto& sampleable : mSampleables)
    generators.emplace_back(sampleable->createSampleGenerator());

  return make_unique<SequentialSampleGenerator>(
      mStateSpace, std::move(generators));
}

} // namespace constraint
} // namespace aikido
