#include <cmath>
#include <aikido/constraint/uniform/CompoundStateSpaceSampleableConstraint.hpp>
#include <dart/common/StlHelpers.h>

using dart::common::make_unique;

namespace aikido {
namespace statespace {

//=============================================================================
CompoundStateSpaceSampleGenerator::CompoundStateSpaceSampleGenerator(
      std::shared_ptr<statespace::CompoundStateSpace> _space,
      std::unique_ptr<util::RNG> _rng,
      std::vector<std::unique_ptr<constraint::SampleGenerator>> _delegates)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mDelegates(std::move(_delegates))
{
}

//=============================================================================
statespace::StateSpacePtr
  CompoundStateSpaceSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool CompoundStateSpaceSampleGenerator::sample(
  statespace::StateSpace::State* _state)
{
  for (size_t i = 0; i < mSpace->getNumStates(); ++i)
  {
    if (!mDelegates[i]->sample(mSpace->getSubState<>(_state, i)))
      return false;
  }
  return true;
}

//=============================================================================
int CompoundStateSpaceSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool CompoundStateSpaceSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
CompoundStateSpaceSampleableConstraint
  ::CompoundStateSpaceSampleableConstraint(
      std::shared_ptr<statespace::CompoundStateSpace> _space,
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
{
  // Use our RNG to generate an initial set of (badly correlated) seeds.
  // TODO: Should we use more seeds here? There is no reason to use the same
  // number of seeds to the std::seed_seq as we generate from it.
  std::vector<util::RNG::result_type> initialSeeds;
  initialSeeds.reserve(mSpace->getNumStates());

  for (size_t i = 0; i < mSpace->getNumStates(); ++i)
    initialSeeds.emplace_back((*mRng)());

  // Use seed_seq to improve the quality of our seeds.
  std::seed_seq seqSeeds(initialSeeds.begin(), initialSeeds.end());
  std::vector<util::RNG::result_type> improvedSeeds(mSpace->getNumStates());
  seqSeeds.generate(improvedSeeds.begin(), improvedSeeds.end());

  throw std::runtime_error(
    "CompoundStateSpaceSampleableConstraint is not implemented.");
#if 0
  // Create a new RNG for each delegate SampleableConstraint.
  mDelegates.reserve(mSpace->getNumStates());
  for (size_t i = 0; i < mSpace->getNumStates(); ++i)
  {
    const auto subspace = mSpace->getSubSpace<>(i);
    mDelegates.emplace_back(subspace->createSampleableConstraint(
      mRng->clone(improvedSeeds[i])));
  }
#endif
}

//=============================================================================
statespace::StateSpacePtr CompoundStateSpaceSampleableConstraint
  ::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  CompoundStateSpaceSampleableConstraint::createSampleGenerator() const
{
  std::vector<std::unique_ptr<constraint::SampleGenerator>> delegates;
  delegates.reserve(mDelegates.size());

  for (const auto& delegateFactory : mDelegates)
    delegates.emplace_back(delegateFactory->createSampleGenerator());

  return std::unique_ptr<CompoundStateSpaceSampleGenerator>(
    new CompoundStateSpaceSampleGenerator(
      mSpace, mRng->clone(), std::move(delegates)));
}

} // namespace statespace
} // namespace aikido
