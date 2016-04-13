#include <dart/common/StlHelpers.h>
#include <aikido/constraint/SampleableSubSpace.h>

namespace aikido {
namespace constraint {

using dart::common::make_unique;

//=============================================================================
class SubSpaceSampleGenerator : public SampleGenerator
{
public:
  SubSpaceSampleGenerator(
        std::shared_ptr<statespace::CompoundStateSpace> _stateSpace,
        std::vector<std::unique_ptr<SampleGenerator>> _generators)
    : mStateSpace(std::move(_stateSpace))
    , mGenerators(std::move(_generators))
  {
  }

  statespace::StateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  bool sample(statespace::StateSpace::State* _state) override
  {
    if (mGenerators.empty())
      return false;

    auto state = static_cast<statespace::CompoundStateSpace::State*>(_state);

    for (size_t i = 0; i < mStateSpace->getNumStates(); ++i)
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
  std::shared_ptr<statespace::CompoundStateSpace> mStateSpace;
  std::vector<std::unique_ptr<SampleGenerator>> mGenerators;
};

//=============================================================================
SampleableSubSpace::SampleableSubSpace(
      std::shared_ptr<statespace::CompoundStateSpace> _stateSpace,
      std::vector<std::shared_ptr<SampleableConstraint>> _constraints)
  : mStateSpace(std::move(_stateSpace))
  , mConstraints(std::move(_constraints))
{
  if (mConstraints.size() != mStateSpace->getNumStates())
  {
    std::stringstream msg;
    msg << "Mismatch between size of CompoundStateSpace and the number of"
        << " constraints: " << mStateSpace->getNumStates() << " != "
        << mConstraints.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  for (size_t i = 0; i < mStateSpace->getNumStates(); ++i)
  {
    if (mConstraints[i]->getStateSpace() != mStateSpace->getSubSpace<>(i))
    {
      std::stringstream msg;
      msg << "Constraint " << i << " is not defined over this StateSpace.";
      throw std::invalid_arugment(msg.str());
    }
  }
}

//=============================================================================
statespace::StateSpacePtr SampleableSubSpace::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
std::unique_ptr<SampleGenerator> SampleableSubSpace
  ::createSampleGenerator() const
{
  std::vector<std::unique_ptr<SampleGenerator>> generators;
  generators.reserve(mStateSpace->getNumStates());

  for (const auto& constraint : mConstraints)
    generators.emplace_back(constraint->createSampleGenerator());

  return make_unique<SubSpaceSampleGenerator>(
    mStateSpace, std::move(generators));
}

} // namespace constraint
} // namespace aikido
