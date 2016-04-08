#include <cmath>
#include <aikido/statespace/SE2StateSpaceSampleableConstraint.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SE2StateSpaceSampleGenerator::SE2StateSpaceSampleGenerator(
      std::shared_ptr<statespace::SE2StateSpace> _space,
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mDistributionX(_space->getTranslationalBounds()(0, 0),
                   _space->getTranslationalBounds()(0, 1))
  , mDistributionY(_space->getTranslationalBounds()(1, 0),
                   _space->getTranslationalBounds()(1, 1))
  , mDistributionAngle(-M_PI, M_PI)
{
  const auto bounds = _space->getTranslationalBounds();

  for (size_t i = 0; i < bounds.rows(); ++i)
  {
    if (!std::isfinite(bounds(i, 0)) || !std::isfinite(bounds(i, 1)))
    {
      std::stringstream msg;
      msg << "Unable to sample from StateSpace because dimension "
          << i << " is unbounded.";
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
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
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
      mSpace, mRng->clone()));
}

} // namespace statespace
} // namespace aikido
