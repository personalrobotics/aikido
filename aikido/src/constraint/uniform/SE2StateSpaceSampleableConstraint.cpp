#include <cmath>
#include <aikido/constraint/uniform/SE2StateSpaceSampleableConstraint.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SE2StateSpaceSampleGenerator::SE2StateSpaceSampleGenerator(
      std::shared_ptr<statespace::SE2StateSpace> _space,
      std::unique_ptr<util::RNG> _rng,
      const Eigen::Vector2d& _lowerTranslationLimits,
      const Eigen::Vector2d& _upperTranslationLimits)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mDistributionX(_lowerTranslationLimits[0], _upperTranslationLimits[0])
  , mDistributionY(_lowerTranslationLimits[1], _upperTranslationLimits[1])
  , mDistributionAngle(-M_PI, M_PI)
{
  for (size_t i = 0; i < 2; ++i)
  {
    if (!std::isfinite(_lowerTranslationLimits[i])
     || !std::isfinite(_upperTranslationLimits[i]))
    {
      std::stringstream msg;
      msg << "Unable to sample from StateSpace because dimension "
          << i << " is unbounded.";
      throw std::runtime_error(msg.str());
    }

    if (_lowerTranslationLimits[i] > _upperTranslationLimits[i])
    {
      std::stringstream msg;
      msg << "Lower limit exceeds upper limit for dimension " << i << ": "
          << _lowerTranslationLimits[i] << " > "
          << _upperTranslationLimits[i] << ".";
      throw std::runtime_error(msg.str());
    }
  }
}

//=============================================================================
statespace::StateSpacePtr
  SE2StateSpaceSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool SE2StateSpaceSampleGenerator::sample(
  statespace::StateSpace::State* _state)
{
  using Isometry2d = SE2StateSpace::Isometry2d;

  Isometry2d transform = Isometry2d::Identity();
  transform.rotate(
    Eigen::Rotation2Dd(mDistributionAngle(*mRng)));
  transform.pretranslate(Eigen::Vector2d(
    mDistributionX(*mRng), mDistributionY(*mRng)));

  mSpace->setIsometry(static_cast<SE2StateSpace::State*>(_state), transform);

  return true;
}

//=============================================================================
int SE2StateSpaceSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool SE2StateSpaceSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
SE2StateSpaceSampleableConstraint
  ::SE2StateSpaceSampleableConstraint(
      std::shared_ptr<statespace::SE2StateSpace> _space,
      std::unique_ptr<util::RNG> _rng,
      const Eigen::Vector2d& _lowerTranslationLimits,
      const Eigen::Vector2d& _upperTranslationLimits)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mLowerLimits(_lowerTranslationLimits)
  , mUpperLimits(_upperTranslationLimits)
{
}

//=============================================================================
statespace::StateSpacePtr SE2StateSpaceSampleableConstraint
  ::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  SE2StateSpaceSampleableConstraint::createSampleGenerator() const
{
  return std::unique_ptr<SE2StateSpaceSampleGenerator>(
    new SE2StateSpaceSampleGenerator(
      mSpace, mRng->clone(), mLowerLimits, mUpperLimits));
}

} // namespace statespace
} // namespace aikido
