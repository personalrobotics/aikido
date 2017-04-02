#include <stdexcept>
#include <aikido/constraint/uniform/ConstantSampler.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
class ConstantSamplerSampleGenerator
  : public constraint::SampleGenerator
{
public:
  statespace::StateSpacePtr getStateSpace() const override;

  bool sample(statespace::StateSpace::State* _state) override;

  int getNumSamples() const override;

  bool canSample() const override;

private:
  ConstantSamplerSampleGenerator(
    std::shared_ptr<statespace::Rn> _space,
    const Eigen::VectorXd& _value);

  std::shared_ptr<statespace::Rn> mSpace;
  Eigen::VectorXd mValue;

  friend class ConstantSampler;
};

//=============================================================================
ConstantSamplerSampleGenerator::ConstantSamplerSampleGenerator(
      std::shared_ptr<statespace::Rn> _space,
      const Eigen::VectorXd& _value)
  : mSpace(std::move(_space))
  , mValue(_value)
{
}

//=============================================================================
statespace::StateSpacePtr
  ConstantSamplerSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool ConstantSamplerSampleGenerator::sample(
  statespace::StateSpace::State* _state)
{
  mSpace->setValue(static_cast<statespace::Rn::State*>(_state), mValue);

  return true;
}

//=============================================================================
int ConstantSamplerSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool ConstantSamplerSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
ConstantSampler
  ::ConstantSampler(
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
statespace::StateSpacePtr ConstantSampler::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  ConstantSampler::createSampleGenerator() const
{
  return std::unique_ptr<ConstantSamplerSampleGenerator>(
    new ConstantSamplerSampleGenerator(
      mSpace, mValue));
}

} // namespace constraint
} // namespace aikido

