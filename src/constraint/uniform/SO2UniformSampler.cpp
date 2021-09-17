#include "aikido/constraint/uniform/SO2UniformSampler.hpp"

#include <cmath>

namespace aikido {
namespace constraint {
namespace uniform {

//==============================================================================
class SO2UniformSampleGenerator : public constraint::SampleGenerator
{
public:
  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  bool sample(statespace::StateSpace::State* _state) override;

  // Documentation inherited.
  int getNumSamples() const override;

  // Documentation inherited.
  bool canSample() const override;

private:
  SO2UniformSampleGenerator(
      std::shared_ptr<const statespace::SO2> _space,
      std::unique_ptr<common::RNG> _rng);

  std::shared_ptr<const statespace::SO2> mSpace;
  std::unique_ptr<common::RNG> mRng;
  std::uniform_real_distribution<double> mDistribution;

  friend class SO2UniformSampler;
};

//==============================================================================
SO2UniformSampleGenerator::SO2UniformSampleGenerator(
    std::shared_ptr<const statespace::SO2> _space,
    std::unique_ptr<common::RNG> _rng)
  : mSpace(std::move(_space)), mRng(std::move(_rng)), mDistribution(-M_PI, M_PI)
{
  // Do nothing
}

//==============================================================================
statespace::ConstStateSpacePtr SO2UniformSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//==============================================================================
bool SO2UniformSampleGenerator::sample(statespace::StateSpace::State* _state)
{
  const double angle = mDistribution(*mRng);
  mSpace->fromAngle(static_cast<statespace::SO2::State*>(_state), angle);
  return true;
}

//==============================================================================
int SO2UniformSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//==============================================================================
bool SO2UniformSampleGenerator::canSample() const
{
  return true;
}

//==============================================================================
SO2UniformSampler::SO2UniformSampler(
    std::shared_ptr<const statespace::SO2> _space,
    std::unique_ptr<common::RNG> _rng)
  : mSpace(std::move(_space)), mRng(std::move(_rng))
{
  if (!mSpace)
    throw std::invalid_argument("StateSpace is null.");

  if (!mRng)
    throw std::invalid_argument("RNG is null.");
}

//==============================================================================
statespace::ConstStateSpacePtr SO2UniformSampler::getStateSpace() const
{
  return mSpace;
}

//==============================================================================
std::unique_ptr<constraint::SampleGenerator>
SO2UniformSampler::createSampleGenerator() const
{
  return std::unique_ptr<SO2UniformSampleGenerator>(
      new SO2UniformSampleGenerator(mSpace, mRng->clone()));
}

} // namespace uniform
} // namespace constraint
} // namespace aikido
