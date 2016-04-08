#include <aikido/statespace/RealVectorStateSpaceSampleableConstraint.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
RealVectorStateSpaceSampleGenerator::RealVectorStateSpaceSampleGenerator(
      std::shared_ptr<statespace::RealVectorStateSpace> _space,
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
{
  mDistributions.reserve(_space->getDimension());
  const auto& bounds = _space->getBounds();

  for (size_t i = 0; i < bounds.rows(); ++i)
  {
    mDistributions.emplace_back(bounds(i, 0), bounds(i, 1));

    if (!std::isfinite(bounds(i, 0)) || !std::isfinite(bounds(i, 1)))
    {
      std::stringstream msg;
      msg << "Unable to sample from StateSpace because dimension "
          << i << " is unbounded.";
      throw std::runtime_error(msg.str());
    }
  }
}

//=============================================================================
statespace::StateSpacePtr
  RealVectorStateSpaceSampleGenerator::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
bool RealVectorStateSpaceSampleGenerator::sample(
  statespace::StateSpace::State* _state)
{
  Eigen::VectorXd value(mDistributions.size());

  for (size_t i = 0; i < value.size(); ++i)
    value[i] = mDistributions[i](*mRng);

  mSpace->setValue(static_cast<RealVectorStateSpace::State*>(_state), value);

  return true;
}

//=============================================================================
int RealVectorStateSpaceSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
bool RealVectorStateSpaceSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
RealVectorStateSpaceSampleableConstraint
  ::RealVectorStateSpaceSampleableConstraint(
      std::shared_ptr<statespace::RealVectorStateSpace> _space,
      std::unique_ptr<util::RNG> _rng)
  : mSpace(std::move(_space))
  , mRng(std::move(_rng))
{
}

//=============================================================================
statespace::StateSpacePtr RealVectorStateSpaceSampleableConstraint
  ::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
std::unique_ptr<constraint::SampleGenerator>
  RealVectorStateSpaceSampleableConstraint::createSampleGenerator() const
{
  return std::unique_ptr<RealVectorStateSpaceSampleGenerator>(
    new RealVectorStateSpaceSampleGenerator(
      mSpace, mRng->clone()));
}

} // namespace statespace
} // namespace aikido
