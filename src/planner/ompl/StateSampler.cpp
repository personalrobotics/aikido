#include "aikido/planner/ompl/GeometricStateSpace.hpp"
#include "aikido/planner/ompl/StateSampler.hpp"

namespace aikido {
namespace planner {
namespace ompl {

//==============================================================================
StateSampler::StateSampler(
    const ::ompl::base::StateSpace* _space,
    std::unique_ptr<aikido::constraint::SampleGenerator> _generator)
  : ::ompl::base::StateSampler(_space), mGenerator(std::move(_generator))
{
  if (_space == nullptr)
  {
    throw std::invalid_argument("StateSpace is nullptr");
  }

  if (mGenerator == nullptr)
  {
    throw std::invalid_argument("Generator is nullptr");
  }
}

//==============================================================================
void StateSampler::sampleUniform(::ompl::base::State* _state)
{
  auto state = static_cast<GeometricStateSpace::StateType*>(_state);

  bool valid = false;
  if (mGenerator->canSample())
  {
    valid = mGenerator->sample(state->mState);
  }

  state->mValid = valid;
}

//==============================================================================
void StateSampler::sampleUniformNear(
    ::ompl::base::State* /*_state*/,
    const ::ompl::base::State* /*_near*/,
    double /*_distance*/)
{
  throw std::runtime_error("sampleUniformNear not implemented.");
}

//==============================================================================
void StateSampler::sampleGaussian(
    ::ompl::base::State* /*_state*/,
    const ::ompl::base::State* /*_mean*/,
    double /*_stdDev*/)
{
  throw std::runtime_error("sampleGaussian not implemented");
}

} // namespace ompl
} // namespace planner
} // namespace aikido
