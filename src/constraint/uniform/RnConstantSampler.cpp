#include <aikido/constraint/uniform/RnConstantSampler.hpp>

#include <stdexcept>

#include <dart/dart.hpp>

namespace aikido {
namespace constraint {

namespace {

//=============================================================================
class RnConstantSamplerSampleGenerator : public constraint::SampleGenerator
{
public:
  RnConstantSamplerSampleGenerator(
    std::shared_ptr<statespace::Rn> _space,
    const Eigen::VectorXd& _value);

  statespace::StateSpacePtr getStateSpace() const override;

  bool sample(statespace::StateSpace::State* _state) override;

  int getNumSamples() const override;

  bool canSample() const override;

private:
  std::shared_ptr<statespace::Rn> mSpace;
  Eigen::VectorXd mValue;
};

//=============================================================================
RnConstantSamplerSampleGenerator::RnConstantSamplerSampleGenerator(
      std::shared_ptr<statespace::Rn> _space,
      const Eigen::VectorXd& _value)
  : mSpace(std::move(_space))
  , mValue(_value)
{
  // Do nothing
}

//=============================================================================
statespace::StateSpacePtr
RnConstantSamplerSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool RnConstantSamplerSampleGenerator::sample(
    statespace::StateSpace::State* _state)
{
  mSpace->setValue(static_cast<statespace::Rn::State*>(_state), mValue);

  return true;
}

//=============================================================================
int RnConstantSamplerSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool RnConstantSamplerSampleGenerator::canSample() const
{
  return true;
}

} // namespace anonymous

//=============================================================================
RnConstantSampler::RnConstantSampler(
    std::shared_ptr<statespace::Rn> _space,
    const Eigen::VectorXd& _value)
  : mSpace(std::move(_space))
  , mValue(_value)
{
  if (!mSpace)
    throw std::invalid_argument("StateSpace is null.");

  if (mSpace->getDimension() != mValue.size())
  {
    std::stringstream msg;
    msg << "Value has incorrect dimension: expected "
        << mSpace->getDimension() << ", got " << mValue.size() << ".";
    throw std::invalid_argument(msg.str());
  }
}

//=============================================================================
statespace::StateSpacePtr RnConstantSampler::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  RnConstantSampler::createSampleGenerator() const
{
  return dart::common::make_unique<RnConstantSamplerSampleGenerator>(
      mSpace, mValue);
}

//=============================================================================
const Eigen::VectorXd& RnConstantSampler::getConstantValue() const
{
  return mValue;
}

} // namespace constraint
} // namespace aikido

