#include <aikido/ompl/AIKIDOStateSampler.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>

namespace aikido {
namespace ompl {

AIKIDOStateSampler::AIKIDOStateSampler(
    const ::ompl::base::StateSpace *_space,
    std::unique_ptr<aikido::constraint::SampleGenerator> _generator)
    : ::ompl::base::StateSampler(_space), mGenerator(std::move(_generator)) {}

/// Sample a state uniformly from the space
void AIKIDOStateSampler::sampleUniform(::ompl::base::State *_state) {
  auto state = static_cast<AIKIDOGeometricStateSpace::StateType *>(_state);

  bool valid = false;
  if (mGenerator->canSample()) {
    valid = mGenerator->sample(state->mState);
  }

  if (!valid) {
    throw std::domain_error("Failed to generate valid sample.");
  }
}

/// Sample a state near another, within specified distance
void AIKIDOStateSampler::sampleUniformNear(::ompl::base::State *_state,
                                           const ::ompl::base::State *_near,
                                           const double _distance) {
  throw std::runtime_error("sampleUniformNear not implemented.");
}

/// Sample a state using a Gaussian distribution with given mean and standard
/// deviation
void AIKIDOStateSampler::sampleGaussian(::ompl::base::State *_state,
                                        const ::ompl::base::State *_mean,
                                        const double _stdDev) {
  throw std::runtime_error("sampleGaussian not implemented");
}
}
}
