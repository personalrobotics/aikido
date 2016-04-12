#include <cmath>
#include <aikido/constraint/uniform/SE3TranslationalConstraint.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
class SE3StateSpaceSampleGenerator
  : public constraint::SampleGenerator
{
public:
  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  bool sample(statespace::StateSpace::State* _state) override;

  // Documentation inherited.
  int getNumSamples() const override;

  // Documentation inherited.
  bool canSample() const override;

private:
  SE3StateSpaceSampleGenerator(
    std::shared_ptr<statespace::SE3StateSpace> _space,
    std::unique_ptr<util::RNG> _rng,
    const Eigen::Vector3d& _lowerTranslationLimits,
    const Eigen::Vector3d& _upperTranslationLimits);

  std::shared_ptr<statespace::SE3StateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::array<
    std::uniform_real_distribution<double>, 3> mTranslationDistributions;
  std::uniform_real_distribution<double> mOrientationDistribution;

  friend class SE3StateSpaceSampleableConstraint;
};

//=============================================================================
SE3StateSpaceSampleGenerator::SE3StateSpaceSampleGenerator(
      std::shared_ptr<statespace::SE3StateSpace> _space,
      std::unique_ptr<util::RNG> _rng,
      const Eigen::Vector3d& _lowerTranslationLimits,
      const Eigen::Vector3d& _upperTranslationLimits)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mOrientationDistribution(0., 1.)
{
  for (size_t i = 0; i < 3; ++i)
  {
    if (!std::isfinite(_lowerTranslationLimits[i])
     || !std::isfinite(_upperTranslationLimits[i]))
    {
      std::stringstream msg;
      msg << "Unable to sample from SE3StateSpace because translational"
             " dimension " << i << " is unbounded.";
      throw std::runtime_error(msg.str());
    }

    if (_lowerTranslationLimits[i] > _upperTranslationLimits[i])
    {
      std::stringstream msg;
      msg << "Lower limit exceeds upper limit for tarnslational dimension "
          << i << ": " << _lowerTranslationLimits[i] << " > "
          << _upperTranslationLimits[i] << ".";
      throw std::runtime_error(msg.str());
    }

    mTranslationDistributions[i] = std::uniform_real_distribution<double>(
      _lowerTranslationLimits[i], _upperTranslationLimits[i]);
  }
}

//=============================================================================
statespace::StateSpacePtr
  SE3StateSpaceSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool SE3StateSpaceSampleGenerator::sample(
  statespace::StateSpace::State* _state)
{
  using Isometry3d = SE3StateSpace::Isometry3d;

  Isometry3d pose = Isometry3d::Identity();
  pose.rotate(util::sampleQuaternion(*mRng, mOrientationDistribution));

  for (size_t i = 0; i < 3; ++i)
    pose.translation()[i] = mTranslationDistributions[i](*mRng);

  mSpace->setIsometry(static_cast<SE3StateSpace::State*>(_state), pose);

  return true;
}

//=============================================================================
int SE3StateSpaceSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool SE3StateSpaceSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
SE3StateSpaceSampleableConstraint
  ::SE3StateSpaceSampleableConstraint(
      std::shared_ptr<statespace::SE3StateSpace> _space,
      std::unique_ptr<util::RNG> _rng,
      const Eigen::Vector3d& _lowerTranslationLimits,
      const Eigen::Vector3d& _upperTranslationLimits)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mLowerLimits(_lowerTranslationLimits)
  , mUpperLimits(_upperTranslationLimits)
{
}

//=============================================================================
statespace::StateSpacePtr SE3StateSpaceSampleableConstraint
  ::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  SE3StateSpaceSampleableConstraint::createSampleGenerator() const
{
  return std::unique_ptr<SE3StateSpaceSampleGenerator>(
    new SE3StateSpaceSampleGenerator(
      mSpace, mRng->clone(), mLowerLimits, mUpperLimits));
}

} // namespace statespace
} // namespace aikido
