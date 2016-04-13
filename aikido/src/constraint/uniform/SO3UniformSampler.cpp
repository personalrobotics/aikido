#include <cmath>
#include <aikido/constraint/uniform/SO3UniformSampler.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
class SO3UniformSampleGenerator : public constraint::SampleGenerator
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
  SO3UniformSampleGenerator(
    std::shared_ptr<statespace::SO3StateSpace> _space,
    std::unique_ptr<util::RNG> _rng);

  std::shared_ptr<statespace::SO3StateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::uniform_real_distribution<double> mDistribution; 

  friend class SO3UniformSampler;
};

//=============================================================================
SO3UniformSampleGenerator::SO3UniformSampleGenerator(
      std::shared_ptr<statespace::SO3StateSpace> _space,
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mDistribution(0., 1.)
{
}

//=============================================================================
statespace::StateSpacePtr
  SO3UniformSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool SO3UniformSampleGenerator::sample(
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
int SO3UniformSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool SO3UniformSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
SO3UniformSampler
  ::SO3UniformSampler(
      std::shared_ptr<statespace::SO3StateSpace> _space,
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
statespace::StateSpacePtr SO3UniformSampler
  ::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  SO3UniformSampler::createSampleGenerator() const
{
  return std::unique_ptr<SO3UniformSampleGenerator>(
    new SO3UniformSampleGenerator(
      mSpace, mRng->clone()));
}

} // namespace statespace
} // namespace aikido
