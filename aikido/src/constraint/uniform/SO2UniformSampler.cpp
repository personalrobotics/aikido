#include <cmath>
#include <aikido/constraint/uniform/SO2UniformSampler.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
class SO2UniformSampleGenerator : public constraint::SampleGenerator
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
  SO2UniformSampleGenerator(
    std::shared_ptr<statespace::SO2> _space,
    std::unique_ptr<util::RNG> _rng);

  std::shared_ptr<statespace::SO2> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::uniform_real_distribution<double> mDistribution; 

  friend class SO2Sampleable;
};

//=============================================================================
SO2UniformSampleGenerator::SO2UniformSampleGenerator(
      std::shared_ptr<statespace::SO2> _space,
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mDistribution(-M_PI, M_PI)
{
}

//=============================================================================
statespace::StateSpacePtr
  SO2UniformSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool SO2UniformSampleGenerator::sample(
  statespace::StateSpace::State* _state)
{
  const double angle = mDistribution(*mRng);
  mSpace->setAngle(static_cast<SO2::State*>(_state), angle);
  return true;
}

//=============================================================================
int SO2UniformSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool SO2UniformSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
SO2Sampleable
  ::SO2Sampleable(
      std::shared_ptr<statespace::SO2> _space,
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
statespace::StateSpacePtr SO2Sampleable
  ::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  SO2Sampleable::createSampleGenerator() const
{
  return std::unique_ptr<SO2UniformSampleGenerator>(
    new SO2UniformSampleGenerator(
      mSpace, mRng->clone()));
}

} // namespace statespace
} // namespace aikido
