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
  const double u1 = mDistribution(*mRng);
  const double u2 = mDistribution(*mRng);
  const double u3 = mDistribution(*mRng);

  mSpace->setQuaternion(
    static_cast<statespace::SO3StateSpace::State*>(_state),
    statespace::SO3StateSpace::Quaternion(
      std::sqrt(1. - u1) * std::sin(2 * M_PI * u2),
      std::sqrt(1. - u1) * std::cos(2 * M_PI * u2),
      std::sqrt(u1) * std::sin(2 * M_PI * u3),
      std::sqrt(u1) * std::cos(2 * M_PI * u3)
  ));
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
