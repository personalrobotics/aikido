#include <aikido/constraint/uniform/SE2BoxConstraint.hpp>

#include <stdexcept>
#include <aikido/constraint/uniform/SO2UniformSampler.hpp>
#include <aikido/statespace/SO2.hpp>

namespace aikido {
namespace constraint {

using constraint::ConstraintType;

//==============================================================================
class SE2BoxConstraintSampleGenerator : public constraint::SampleGenerator
{
public:
  statespace::StateSpacePtr getStateSpace() const override;

  bool sample(statespace::StateSpace::State* _state) override;

  int getNumSamples() const override;

  bool canSample() const override;

private:
  SE2BoxConstraintSampleGenerator(
      std::shared_ptr<statespace::SE2> _space,
      std::unique_ptr<common::RNG> _rng,
      const Eigen::Vector3d& _lowerLimits,
      const Eigen::Vector3d& _upperLimits);

  const std::size_t mDimension;
  std::shared_ptr<statespace::SE2> mSpace;
  std::unique_ptr<common::RNG> mRng;
  std::vector<std::uniform_real_distribution<double>> mDistributions;
  friend class SE2BoxConstraint;
};

//==============================================================================
SE2BoxConstraintSampleGenerator::SE2BoxConstraintSampleGenerator(
    std::shared_ptr<statespace::SE2> _space,
    std::unique_ptr<common::RNG> _rng,
    const Eigen::Vector3d& _lowerLimits,
    const Eigen::Vector3d& _upperLimits)
  : mDimension(3), mSpace(std::move(_space)), mRng(std::move(_rng))
{
  mDistributions.reserve(mDimension);

  for (std::size_t i = 0; i < mDimension; ++i)
    mDistributions.emplace_back(_lowerLimits[i], _upperLimits[i]);
}

//==============================================================================
statespace::StateSpacePtr SE2BoxConstraintSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//==============================================================================
bool SE2BoxConstraintSampleGenerator::sample(
    statespace::StateSpace::State* _state)
{
  Eigen::Vector3d tangent;

  for (auto i = 0; i < tangent.size(); ++i)
    tangent[i] = mDistributions[i](*mRng);

  mSpace->expMap(tangent, static_cast<statespace::SE2::State*>(_state));

  return true;
}

//==============================================================================
int SE2BoxConstraintSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//==============================================================================
bool SE2BoxConstraintSampleGenerator::canSample() const
{
  return true;
}

//==============================================================================
SE2BoxConstraint::SE2BoxConstraint(
    std::shared_ptr<statespace::SE2> space,
    std::unique_ptr<common::RNG> rng,
    const Eigen::Vector2d& lowerLimits,
    const Eigen::Vector2d& upperLimits)
  : mSpace(std::move(space))
  , mRng(std::move(rng))
  , mRnDimension(2)
  , mDimension(3)
{
  mLowerLimits[0] = -M_PI;
  mUpperLimits[0] = M_PI;
  mLowerLimits.tail<2>() = lowerLimits;
  mUpperLimits.tail<2>() = upperLimits;

  if (!mSpace)
    throw std::invalid_argument("StateSpace is null.");

  if (static_cast<std::size_t>(mLowerLimits.size()) != mDimension)
  {
    std::stringstream msg;
    msg << "Lower limits have incorrect dimension: expected " << mDimension
        << ", got " << mLowerLimits.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  if (static_cast<std::size_t>(mUpperLimits.size()) != mDimension)
  {
    std::stringstream msg;
    msg << "Upper limits have incorrect dimension: expected " << mDimension
        << ", got " << mUpperLimits.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  for (std::size_t i = mDimension - mRnDimension; i < mDimension; ++i)
  {
    if (mLowerLimits[i] > mUpperLimits[i])
    {
      std::stringstream msg;
      msg << "Unable to sample from StateSpace because lower limit exceeds"
          << " upper limit on dimension " << i << ": " << mLowerLimits[i]
          << " > " << mUpperLimits[i] << ".";
      throw std::invalid_argument(msg.str());
    }
  }
}

//==============================================================================
statespace::StateSpacePtr SE2BoxConstraint::getStateSpace() const
{
  return mSpace;
}

//==============================================================================
bool SE2BoxConstraint::isSatisfied(
    const statespace::StateSpace::State* state, TestableOutcome* outcome) const
{
  auto defaultOutcomeObject
      = dynamic_cast_if_present<DefaultTestableOutcome>(outcome);

  Eigen::VectorXd tangent;
  mSpace->logMap(static_cast<const statespace::SE2::State*>(state), tangent);

  for (std::size_t i = mDimension - mRnDimension; i < mDimension; ++i)
  {
    if (tangent[i] < mLowerLimits[i] || tangent[i] > mUpperLimits[i])
    {
      if (defaultOutcomeObject)
        defaultOutcomeObject->setSatisfiedFlag(false);
      return false;
    }
  }

  if (defaultOutcomeObject)
    defaultOutcomeObject->setSatisfiedFlag(true);
  return true;
}

//==============================================================================
std::unique_ptr<TestableOutcome> SE2BoxConstraint::createOutcome() const
{
  return std::unique_ptr<TestableOutcome>(new DefaultTestableOutcome);
}

//==============================================================================
bool SE2BoxConstraint::project(
    const statespace::StateSpace::State* s,
    statespace::StateSpace::State* out) const
{
  Eigen::VectorXd tangent;
  mSpace->logMap(static_cast<const statespace::SE2::State*>(s), tangent);

  for (std::size_t i = mDimension - mRnDimension; i < mDimension; ++i)
  {
    if (tangent[i] < mLowerLimits[i])
      tangent[i] = mLowerLimits[i];
    else if (tangent[i] > mUpperLimits[i])
      tangent[i] = mUpperLimits[i];
  }

  mSpace->expMap(tangent, static_cast<statespace::SE2::State*>(out));

  return true;
}

//==============================================================================
std::unique_ptr<constraint::SampleGenerator>
SE2BoxConstraint::createSampleGenerator() const
{
  if (!mRng)
    throw std::invalid_argument("mRng is null.");

  for (std::size_t i = mDimension - mRnDimension; i < mDimension; ++i)
  {
    if (!(std::isfinite(mLowerLimits[i]) && std::isfinite(mUpperLimits[i])))
    {
      std::stringstream msg;
      msg << "Unable to sample from StateSpace because dimension " << i
          << " is unbounded.";
      throw std::runtime_error(msg.str());
    }
  }

  return std::unique_ptr<SE2BoxConstraintSampleGenerator>(
      new SE2BoxConstraintSampleGenerator(
          mSpace, mRng->clone(), mLowerLimits, mUpperLimits));
}

//==============================================================================
Eigen::Vector2d SE2BoxConstraint::getLowerLimits() const
{
  return mLowerLimits.tail<2>();
}

//==============================================================================
Eigen::Vector2d SE2BoxConstraint::getUpperLimits() const
{
  return mUpperLimits.tail<2>();
}

} // namespace statespace
} // namespace aikido
