#include <dart/common/StlHelpers.hpp>
#include <aikido/constraint/CartesianProductSampleable.hpp>

namespace aikido {
namespace constraint {

using dart::common::make_unique;

//==============================================================================
class SubspaceSampleGenerator : public SampleGenerator
{
public:
  SubspaceSampleGenerator(
      std::shared_ptr<statespace::CartesianProduct> _stateSpace,
      std::vector<std::unique_ptr<SampleGenerator>> _generators)
    : mStateSpace(std::move(_stateSpace)), mGenerators(std::move(_generators))
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
  }

  statespace::StateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  bool sample(statespace::StateSpace::State* _state) override
  {
    if (mGenerators.empty())
      return false;

    auto state = static_cast<statespace::CartesianProduct::State*>(_state);

    for (std::size_t i = 0; i < mStateSpace->getNumSubspaces(); ++i)
    {
      auto subState = mStateSpace->getSubState<>(state, i);

      if (!mGenerators[i]->sample(subState))
        return false;
    }

    return true;
  }

  int getNumSamples() const override
  {
    if (mGenerators.empty())
      return 0;

    int numSamples = std::numeric_limits<int>::max();

    for (const auto& generator : mGenerators)
      numSamples = std::min(numSamples, generator->getNumSamples());

    return numSamples;
  }

  bool canSample() const override
  {
    if (mGenerators.empty())
      return false;

    for (const auto& generator : mGenerators)
    {
      if (!generator->canSample())
        return false;
    }

    return true;
  }

private:
  std::shared_ptr<statespace::CartesianProduct> mStateSpace;
  std::vector<std::unique_ptr<SampleGenerator>> mGenerators;
};

//==============================================================================
CartesianProductSampleable::CartesianProductSampleable(
    std::shared_ptr<statespace::CartesianProduct> _stateSpace,
    std::vector<std::shared_ptr<Sampleable>> _constraints)
  : mStateSpace(std::move(_stateSpace)), mConstraints(std::move(_constraints))
{
  if (!mStateSpace)
    throw std::invalid_argument("_stateSpace is nullptr.");

  if (mConstraints.size() != mStateSpace->getNumSubspaces())
  {
    std::stringstream msg;
    msg << "Mismatch between size of CartesianProduct and the number of"
        << " constraints: " << mStateSpace->getNumSubspaces()
        << " != " << mConstraints.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  for (std::size_t i = 0; i < mStateSpace->getNumSubspaces(); ++i)
  {
    if (!mConstraints[i])
    {
      std::stringstream msg;
      msg << "Constraint " << i << " is null.";
      throw std::invalid_argument(msg.str());
    }

    if (mConstraints[i]->getStateSpace() != mStateSpace->getSubspace<>(i))
    {
      std::stringstream msg;
      msg << "Constraint " << i << " is not defined over this StateSpace.";
      throw std::invalid_argument(msg.str());
    }
  }
}

//==============================================================================
statespace::ConstStateSpacePtr CartesianProductSampleable::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::unique_ptr<SampleGenerator>
CartesianProductSampleable::createSampleGenerator() const
{
  std::vector<std::unique_ptr<SampleGenerator>> generators;
  generators.reserve(mStateSpace->getNumSubspaces());

  for (const auto& constraint : mConstraints)
    generators.emplace_back(constraint->createSampleGenerator());

  return make_unique<SubspaceSampleGenerator>(
      mStateSpace, std::move(generators));
}

} // namespace constraint
} // namespace aikido
