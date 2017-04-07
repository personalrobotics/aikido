#include <stdexcept>
#include <aikido/constraint/uniform/RnBoxConstraint.hpp>

namespace aikido {
namespace constraint {

using constraint::ConstraintType;

//=============================================================================
class RnBoxConstraintSampleGenerator : public constraint::SampleGenerator
{
public:
  statespace::StateSpacePtr getStateSpace() const override;

  bool sample(statespace::StateSpace::State* _state) override;

  int getNumSamples() const override;

  bool canSample() const override;

private:
  RnBoxConstraintSampleGenerator(
    std::shared_ptr<statespace::Rn> _space,
    std::unique_ptr<util::RNG> _rng,
    const Eigen::VectorXd& _lowerLimits,
    const Eigen::VectorXd& _upperLimits);

  std::shared_ptr<statespace::Rn> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::vector<std::uniform_real_distribution<double>> mDistributions;

  friend class RnBoxConstraint;
};

//=============================================================================
RnBoxConstraintSampleGenerator::RnBoxConstraintSampleGenerator(
      std::shared_ptr<statespace::Rn> _space,
      std::unique_ptr<util::RNG> _rng,
      const Eigen::VectorXd& _lowerLimits,
      const Eigen::VectorXd& _upperLimits)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
{
  const auto dimension = mSpace->getDimension();
  mDistributions.reserve(dimension);

  for (size_t i = 0; i < dimension; ++i)
    mDistributions.emplace_back(_lowerLimits[i], _upperLimits[i]);
}

//=============================================================================
statespace::StateSpacePtr
  RnBoxConstraintSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool RnBoxConstraintSampleGenerator::sample(
  statespace::StateSpace::State* _state)
{
  Eigen::VectorXd value(mDistributions.size());

  for (int i = 0; i < value.size(); ++i) 
    value[i] = mDistributions[i](*mRng);

  mSpace->setValue(static_cast<statespace::Rn::State*>(_state), value);

  return true;
}

//=============================================================================
int RnBoxConstraintSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool RnBoxConstraintSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
RnBoxConstraint
  ::RnBoxConstraint(
      std::shared_ptr<statespace::Rn> _space,
      std::unique_ptr<util::RNG> _rng,
      const Eigen::VectorXd& _lowerLimits,
      const Eigen::VectorXd& _upperLimits)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mLowerLimits(_lowerLimits)
  , mUpperLimits(_upperLimits)
{
  if (!mSpace)
    throw std::invalid_argument("StateSpace is null.");

  const auto dimension = mSpace->getDimension();

  if (static_cast<size_t>(mLowerLimits.size()) != dimension) 
  {
    std::stringstream msg;
    msg << "Lower limits have incorrect dimension: expected "
        << mSpace->getDimension() << ", got " << mLowerLimits.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  if (static_cast<size_t>(mUpperLimits.size()) != dimension)
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
statespace::StateSpacePtr RnBoxConstraint::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
size_t RnBoxConstraint::getConstraintDimension() const
{
  // TODO: Only create constraints for bounded dimensions.
  return mSpace->getDimension();
}

//=============================================================================
std::vector<ConstraintType> RnBoxConstraint::getConstraintTypes() const
{
  return std::vector<ConstraintType>(
    mSpace->getDimension(), ConstraintType::INEQUALITY);
}

//=============================================================================
bool RnBoxConstraint::isSatisfied(const statespace::StateSpace::State* state) const
{
  const auto value = mSpace->getValue(
    static_cast<const statespace::Rn::State*>(state));

  for (int i = 0; i < value.size(); ++i)
  {
    if (value[i] < mLowerLimits[i] || value[i] > mUpperLimits[i])
      return false;
  }
  return true;
}

//=============================================================================
bool RnBoxConstraint::project(
  const statespace::StateSpace::State* _s,
  statespace::StateSpace::State* _out) const
{
  Eigen::VectorXd value = mSpace->getValue(
    static_cast<const statespace::Rn::State*>(_s));

  for (int i = 0; i < value.size(); ++i)
  {
    if (value[i] < mLowerLimits[i])
      value[i] = mLowerLimits[i];
    else if (value[i] > mUpperLimits[i])
      value[i] = mUpperLimits[i];
  }

  mSpace->setValue(
    static_cast<statespace::Rn::State*>(_out), value);

  return true;
}

//=============================================================================
void RnBoxConstraint::getValue(
  const statespace::StateSpace::State* _s,
  Eigen::VectorXd& _out) const
{
  auto stateValue = mSpace->getValue(
    static_cast<const statespace::Rn::State*>(_s));

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
void RnBoxConstraint::getJacobian(
  const statespace::StateSpace::State* _s,
  Eigen::MatrixXd& _out) const
{
  auto stateValue = mSpace->getValue(
    static_cast<const statespace::Rn::State*>(_s));

  const size_t dimension = mSpace->getDimension();
  _out = Eigen::MatrixXd::Zero(dimension, dimension);

  for (int i = 0; i < _out.rows(); ++i)
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
std::unique_ptr<constraint::SampleGenerator>
  RnBoxConstraint::createSampleGenerator() const
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

  return std::unique_ptr<RnBoxConstraintSampleGenerator>(
    new RnBoxConstraintSampleGenerator(
      mSpace, mRng->clone(), mLowerLimits, mUpperLimits));
}

//=============================================================================
const Eigen::VectorXd& RnBoxConstraint::getLowerLimits() const
{
  return mLowerLimits;
}

//=============================================================================
const Eigen::VectorXd& RnBoxConstraint::getUpperLimits() const
{
  return mUpperLimits;
}

} // namespace statespace
} // namespace aikido
