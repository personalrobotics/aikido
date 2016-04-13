#include <cmath>
#include <aikido/constraint/uniform/SO2UniformSampler.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
class SO2StateSpaceSampleGenerator : public constraint::SampleGenerator
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
  SO2StateSpaceSampleGenerator(
    std::shared_ptr<statespace::SO2StateSpace> _space,
    std::unique_ptr<util::RNG> _rng);

  std::shared_ptr<statespace::SO2StateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::uniform_real_distribution<double> mDistribution; 

  friend class SO2StateSpaceSampleableConstraint;
};

//=============================================================================
SO2StateSpaceSampleGenerator::SO2StateSpaceSampleGenerator(
      std::shared_ptr<statespace::SO2StateSpace> _space,
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mDistribution(-M_PI, M_PI)
{
}

//=============================================================================
statespace::StateSpacePtr
  SO2StateSpaceSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool SO2StateSpaceSampleGenerator::sample(
  statespace::StateSpace::State* _state)
{
  const double angle = mDistribution(*mRng);
  mSpace->setAngle(static_cast<SO2StateSpace::State*>(_state), angle);
  return true;
}

//=============================================================================
int SO2StateSpaceSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool SO2StateSpaceSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
SO2StateSpaceSampleableConstraint
  ::SO2StateSpaceSampleableConstraint(
      std::shared_ptr<statespace::SO2StateSpace> _space,
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
{
  if (!mSpace)
    throw std::invalid_argument("StateSpace is null.");

  if (!mRng)
    throw std::invalid_argument("RNG is null.");
}

//=============================================================================
statespace::StateSpacePtr SO2StateSpaceSampleableConstraint
  ::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  SO2StateSpaceSampleableConstraint::createSampleGenerator() const
{
  return std::unique_ptr<SO2StateSpaceSampleGenerator>(
    new SO2StateSpaceSampleGenerator(
      mSpace, mRng->clone()));
}

} // namespace statespace
} // namespace aikido
