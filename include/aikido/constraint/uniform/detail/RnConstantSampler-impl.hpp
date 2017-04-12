#include <aikido/constraint/uniform/RnConstantSampler.hpp>

#include <stdexcept>

#include <dart/dart.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
extern template
class RConstantSampler<0>;

extern template
class RConstantSampler<1>;

extern template
class RConstantSampler<2>;

extern template
class RConstantSampler<3>;

extern template
class RConstantSampler<6>;

extern template
class RConstantSampler<Eigen::Dynamic>;

namespace {

//=============================================================================
template <int N>
class RnConstantSamplerSampleGenerator : public constraint::SampleGenerator
{
public:
  using Vectord = Eigen::Matrix<double, N, 1>;

  RnConstantSamplerSampleGenerator(
    std::shared_ptr<statespace::R<N>> _space, const Vectord& _value);

  statespace::StateSpacePtr getStateSpace() const override;

  bool sample(statespace::StateSpace::State* _state) override;

  int getNumSamples() const override;

  bool canSample() const override;

private:
  std::shared_ptr<statespace::R<N>> mSpace;
  Vectord mValue;
};

//=============================================================================
template <int N>
RnConstantSamplerSampleGenerator<N>::RnConstantSamplerSampleGenerator(
      std::shared_ptr<statespace::R<N>> _space,
      const Vectord& _value)
  : mSpace(std::move(_space))
  , mValue(_value)
{
  // Do nothing
}

//=============================================================================
template <int N>
statespace::StateSpacePtr
RnConstantSamplerSampleGenerator<N>::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
template <int N>
bool RnConstantSamplerSampleGenerator<N>::sample(
    statespace::StateSpace::State* _state)
{
  mSpace->setValue(static_cast<typename statespace::R<N>::State*>(_state), mValue);

  return true;
}

//=============================================================================
template <int N>
int RnConstantSamplerSampleGenerator<N>::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
template <int N>
bool RnConstantSamplerSampleGenerator<N>::canSample() const
{
  return true;
}

} // namespace anonymous

//=============================================================================
template <int N>
RConstantSampler<N>::RConstantSampler(
    std::shared_ptr<statespace::R<N>> _space, const Vectord& _value)
  : mSpace(std::move(_space))
  , mValue(_value)
{
  if (!mSpace)
    throw std::invalid_argument("StateSpace is null.");

  if (mSpace->getDimension() != static_cast<std::size_t>(mValue.size()))
  {
    std::stringstream msg;
    msg << "Value has incorrect dimension: expected "
        << mSpace->getDimension() << ", got " << mValue.size() << ".";
    throw std::invalid_argument(msg.str());
  }
}

//=============================================================================
template <int N>
statespace::StateSpacePtr RConstantSampler<N>::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
template <int N>
std::unique_ptr<constraint::SampleGenerator>
  RConstantSampler<N>::createSampleGenerator() const
{
  return dart::common::make_unique<RnConstantSamplerSampleGenerator<N>>(
      mSpace, mValue);
}

//=============================================================================
template <int N>
const typename RConstantSampler<N>::Vectord&
RConstantSampler<N>::getConstantValue() const
{
  return mValue;
}

} // namespace constraint
} // namespace aikido
