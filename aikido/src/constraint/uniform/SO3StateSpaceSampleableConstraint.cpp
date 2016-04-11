#include <cmath>
#include <aikido/statespace/SO3StateSpaceSampleableConstraint.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SO3StateSpaceSampleGenerator::SO3StateSpaceSampleGenerator(
      std::shared_ptr<statespace::SO3StateSpace> _space,
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mDistribution(0., 1.)
{
}

//=============================================================================
statespace::StateSpacePtr
  SO3StateSpaceSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool SO3StateSpaceSampleGenerator::sample(
  statespace::StateSpace::State* _state)
{
  mSpace->setQuaternion(
    static_cast<SO3StateSpace::State*>(_state),
    util::sampleQuaternion<util::RNG, double, SO3StateSpace::Quaternion>(
      *mRng, mDistribution)
  );

  return true;
}

//=============================================================================
int SO3StateSpaceSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool SO3StateSpaceSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
SO3StateSpaceSampleableConstraint
  ::SO3StateSpaceSampleableConstraint(
      std::shared_ptr<statespace::SO3StateSpace> _space,
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
{
}

//=============================================================================
statespace::StateSpacePtr SO3StateSpaceSampleableConstraint
  ::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  SO3StateSpaceSampleableConstraint::createSampleGenerator() const
{
  return std::unique_ptr<SO3StateSpaceSampleGenerator>(
    new SO3StateSpaceSampleGenerator(
      mSpace, mRng->clone()));
}

} // namespace statespace
} // namespace aikido
