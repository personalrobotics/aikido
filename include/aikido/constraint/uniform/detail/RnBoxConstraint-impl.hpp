#include <aikido/constraint/uniform/RnBoxConstraint.hpp>

#include <stdexcept>

namespace aikido {
namespace constraint {

using constraint::ConstraintType;

//==============================================================================
extern template
class RBoxConstraint<0>;

extern template
class RBoxConstraint<1>;

extern template
class RBoxConstraint<2>;

extern template
class RBoxConstraint<3>;

extern template
class RBoxConstraint<6>;

extern template
class RBoxConstraint<Eigen::Dynamic>;

//=============================================================================
template <int N>
class RnBoxConstraintSampleGenerator : public constraint::SampleGenerator
{
public:
  using VectorNd = Eigen::Matrix<double, N, 1>;

  statespace::StateSpacePtr getStateSpace() const override;

  bool sample(statespace::StateSpace::State* _state) override;

  int getNumSamples() const override;

  bool canSample() const override;

private:
  RnBoxConstraintSampleGenerator(std::shared_ptr<statespace::R<N>> _space,
    std::unique_ptr<util::RNG> _rng,
    const VectorNd& _lowerLimits,
    const VectorNd& _upperLimits);

  std::shared_ptr<statespace::R<N>> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::vector<std::uniform_real_distribution<double>> mDistributions;

  friend class RBoxConstraint<N>;
};

//=============================================================================
template <int N>
RnBoxConstraintSampleGenerator<N>::RnBoxConstraintSampleGenerator(
      std::shared_ptr<statespace::R<N>> _space,
      std::unique_ptr<util::RNG> _rng,
      const VectorNd& _lowerLimits,
      const VectorNd& _upperLimits)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
{
  const auto dimension = mSpace->getDimension();
  mDistributions.reserve(dimension);

  for (size_t i = 0; i < dimension; ++i)
    mDistributions.emplace_back(_lowerLimits[i], _upperLimits[i]);
}

//=============================================================================
template <int N>
statespace::StateSpacePtr
  RnBoxConstraintSampleGenerator<N>::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
template <int N>
bool RnBoxConstraintSampleGenerator<N>::sample(
  statespace::StateSpace::State* _state)
{
  VectorNd value(mDistributions.size());

  for (auto i = 0; i < value.size(); ++i)
    value[i] = mDistributions[i](*mRng);

  mSpace->setValue(static_cast<typename statespace::R<N>::State*>(_state), value);

  return true;
}

//=============================================================================
template <int N>
int RnBoxConstraintSampleGenerator<N>::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
template <int N>
bool RnBoxConstraintSampleGenerator<N>::canSample() const
{
  return true;
}

//=============================================================================
template <int N>
RBoxConstraint<N>::RBoxConstraint(std::shared_ptr<statespace::R<N>> _space,
      std::unique_ptr<util::RNG> _rng,
      const VectorNd& _lowerLimits,
      const VectorNd& _upperLimits)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mLowerLimits(_lowerLimits)
  , mUpperLimits(_upperLimits)
{
  if (!mSpace)
    throw std::invalid_argument("StateSpace is null.");

  const auto dimension = mSpace->getDimension();

  if (static_cast<std::size_t>(mLowerLimits.size()) != dimension)
  {
    std::stringstream msg;
    msg << "Lower limits have incorrect dimension: expected "
        << mSpace->getDimension() << ", got " << mLowerLimits.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  if (static_cast<std::size_t>(mUpperLimits.size()) != dimension)
  {
    std::stringstream msg;
    msg << "Upper limits have incorrect dimension: expected "
        << mSpace->getDimension() << ", got " << mUpperLimits.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  for (size_t i = 0; i < dimension; ++i)
  {
    if (mLowerLimits[i] > mUpperLimits[i])
    {
      std::stringstream msg;
      msg << "Unable to sample from StateSpace because lower limit exeeds"
             " upper limit on dimension " << i << ": "
          << mLowerLimits[i] << " > " << mUpperLimits[i] << ".";
      throw std::invalid_argument(msg.str());
    }
  }
}

//=============================================================================
template <int N>
statespace::StateSpacePtr RBoxConstraint<N>::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
template <int N>
size_t RBoxConstraint<N>::getConstraintDimension() const
{
  // TODO: Only create constraints for bounded dimensions.
  return mSpace->getDimension();
}

//=============================================================================
template <int N>
std::vector<ConstraintType> RBoxConstraint<N>::getConstraintTypes() const
{
  return std::vector<ConstraintType>(
    mSpace->getDimension(), ConstraintType::INEQUALITY);
}

//=============================================================================
template <int N>
bool RBoxConstraint<N>::isSatisfied(
        const statespace::StateSpace::State* state) const
{
  const auto value = mSpace->getValue(
    static_cast<const typename statespace::R<N>::State*>(state));

  for (auto i = 0; i < value.size(); ++i)
  {
    if (value[i] < mLowerLimits[i] || value[i] > mUpperLimits[i])
      return false;
  }
  return true;
}

//=============================================================================
template <int N>
bool RBoxConstraint<N>::project(
  const statespace::StateSpace::State* _s,
  statespace::StateSpace::State* _out) const
{
  VectorNd value = mSpace->getValue(
    static_cast<const typename statespace::R<N>::State*>(_s));

  for (auto i = 0; i < value.size(); ++i)
  {
    if (value[i] < mLowerLimits[i])
      value[i] = mLowerLimits[i];
    else if (value[i] > mUpperLimits[i])
      value[i] = mUpperLimits[i];
  }

  mSpace->setValue(
    static_cast<typename statespace::R<N>::State*>(_out), value);

  return true;
}

//=============================================================================
template <int N>
void RBoxConstraint<N>::getValue(
  const statespace::StateSpace::State* _s,
  Eigen::VectorXd& _out) const
{
  auto stateValue = mSpace->getValue(
    static_cast<const typename statespace::R<N>::State*>(_s));

  const size_t dimension = mSpace->getDimension();
  _out.resize(dimension);

  for (size_t i = 0; i < dimension; ++i)
  {
    if (stateValue[i] < mLowerLimits[i])
      _out[i] = stateValue[i] - mLowerLimits[i];
    else if (stateValue[i] > mUpperLimits[i])
      _out[i] = mUpperLimits[i] - stateValue[i];
    else
      _out[i] = 0.;
  }
}

//=============================================================================
template <int N>
void RBoxConstraint<N>::getJacobian(
  const statespace::StateSpace::State* _s,
  Eigen::MatrixXd& _out) const
{
  auto stateValue = mSpace->getValue(
    static_cast<const typename statespace::R<N>::State*>(_s));

  const size_t dimension = mSpace->getDimension();
  _out = Eigen::MatrixXd::Zero(dimension, dimension);

  for (auto i = 0; i < _out.rows(); ++i)
  {
    if (stateValue[i] < mLowerLimits[i])
      _out(i, i) = -1.;
    else if (stateValue[i] > mUpperLimits[i])
      _out(i, i) =  1.;
    else
      _out(i, i) =  0.;
  }
}

//=============================================================================
template <int N>
std::unique_ptr<constraint::SampleGenerator>
  RBoxConstraint<N>::createSampleGenerator() const
{
  if (!mRng)
    throw std::invalid_argument("mRng is null.");

  for (size_t i = 0; i < mSpace->getDimension(); ++i)
  {
    if (!std::isfinite(mLowerLimits[i]) || !std::isfinite(mUpperLimits[i]))
    {
      std::stringstream msg;
      msg << "Unable to sample from StateSpace because dimension "
          << i << " is unbounded.";
      throw std::runtime_error(msg.str());
    }
  }

  return std::unique_ptr<RnBoxConstraintSampleGenerator<N>>(
    new RnBoxConstraintSampleGenerator<N>(
      mSpace, mRng->clone(), mLowerLimits, mUpperLimits));
}

//=============================================================================
template <int N>
auto RBoxConstraint<N>::getLowerLimits() const -> const VectorNd&
{
  return mLowerLimits;
}

//=============================================================================
template <int N>
auto RBoxConstraint<N>::getUpperLimits() const -> const VectorNd&
{
  return mUpperLimits;
}

} // namespace statespace
} // namespace aikido
