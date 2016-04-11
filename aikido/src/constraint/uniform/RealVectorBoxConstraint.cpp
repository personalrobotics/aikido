#include <aikido/constraint/uniform/RealVectorBoxConstraint.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
class RealVectorBoxConstraintSampleGenerator
  : public constraint::SampleGenerator
{
public:
  statespace::StateSpacePtr getStateSpace() const override;

  bool sample(statespace::StateSpace::State* _state) override;

  int getNumSamples() const override;

  bool canSample() const override;

private:
  RealVectorBoxConstraintSampleGenerator(
    std::shared_ptr<statespace::RealVectorStateSpace> _space,
    std::unique_ptr<util::RNG> _rng,
    const Eigen::VectorXd& _lowerLimits,
    const Eigen::VectorXd& _upperLimits);

  std::shared_ptr<statespace::RealVectorStateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::vector<std::uniform_real_distribution<double>> mDistributions;

  friend class RealVectorBoxConstraint;
};

//=============================================================================
RealVectorBoxConstraintSampleGenerator::RealVectorBoxConstraintSampleGenerator(
      std::shared_ptr<statespace::RealVectorStateSpace> _space,
      std::unique_ptr<util::RNG> _rng,
      const Eigen::VectorXd& _lowerLimits,
      const Eigen::VectorXd& _upperLimits)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
{
  const auto dimension = _space->getDimension();

  mDistributions.reserve(dimension);

  if (_lowerLimits.size() != dimension)
  {
    std::stringstream msg;
    msg << "Lower limits have incorrect dimension: expected "
        << _space->getDimension() << ", got " << _lowerLimits.size() << ".";
    throw std::runtime_error(msg.str());
  }

  if (_upperLimits.size() != dimension)
  {
    std::stringstream msg;
    msg << "Upper limits have incorrect dimension: expected "
        << _space->getDimension() << ", got " << _upperLimits.size() << ".";
    throw std::runtime_error(msg.str());
  }

  for (size_t i = 0; i < dimension; ++i)
  {
    if (!std::isfinite(_lowerLimits[i]) || !std::isfinite(_upperLimits[i]))
    {
      std::stringstream msg;
      msg << "Unable to sample from StateSpace because dimension "
          << i << " is unbounded.";
      throw std::runtime_error(msg.str());
    }

    if (_lowerLimits[i] > _upperLimits[i])
    {
      std::stringstream msg;
      msg << "Unable to sample from StateSpace because lower limit exeeds"
             " upper limit on dimension " << i << ": "
          << _lowerLimits[i] << " > " << _upperLimits[i] << ".";
      throw std::runtime_error(msg.str());
    }

    mDistributions.emplace_back(_lowerLimits[i], _upperLimits[i]);
  }
}

//=============================================================================
statespace::StateSpacePtr
  RealVectorBoxConstraintSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool RealVectorBoxConstraintSampleGenerator::sample(
  statespace::StateSpace::State* _state)
{
  Eigen::VectorXd value(mDistributions.size());

  for (size_t i = 0; i < value.size(); ++i)
    value[i] = mDistributions[i](*mRng);

  mSpace->setValue(static_cast<RealVectorStateSpace::State*>(_state), value);

  return true;
}

//=============================================================================
int RealVectorBoxConstraintSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool RealVectorBoxConstraintSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
RealVectorBoxConstraint
  ::RealVectorBoxConstraint(
      std::shared_ptr<statespace::RealVectorStateSpace> _space,
      std::unique_ptr<util::RNG> _rng,
      const Eigen::VectorXd& _lowerLimits,
      const Eigen::VectorXd& _upperLimits)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
  , mLowerLimits(_lowerLimits)
  , mUpperLimits(_upperLimits)
{
}

//=============================================================================
statespace::StateSpacePtr RealVectorBoxConstraint
  ::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  RealVectorBoxConstraint::createSampleGenerator() const
{
  return std::unique_ptr<RealVectorBoxConstraintSampleGenerator>(
    new RealVectorBoxConstraintSampleGenerator(
      mSpace, mRng->clone(), mLowerLimits, mUpperLimits));
}

} // namespace statespace
} // namespace aikido
