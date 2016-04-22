#ifndef AIKIDO_OMPL_AIKIDOSTATESAMPLER_HPP_
#define AIKIDO_OMPL_AIKIDOSTATESAMPLER_HPP_

#include <ompl/base/StateSampler.h>
#include "../constraint/Sampleable.hpp"

namespace aikido {
namespace ompl {

/// Wraps an aikido::constraint::SampleGenerator in a
/// ompl::base::StateSampler
class StateSampler : public ::ompl::base::StateSampler {

public:
  /// Constructor
  /// \param _space The OMPL StateSpace this sampler is defined against
  /// \param _generator A SampleGenerator capable of generating samples for the
  /// aikido::statespace::StateSpace wrapped by _space
  StateSampler(
      const ::ompl::base::StateSpace *_space,
      std::unique_ptr<constraint::SampleGenerator> _generator);

  /// Sample a state from the space. Warning: The sampling is not guarenteed
  /// uniform.  The distribution of the sampling is determined by the
  /// SampleGenerator wrapped by this class.
  void sampleUniform(::ompl::base::State *_state) override;

  /// Not implemented. Throws std::domain_error
  void sampleUniformNear(::ompl::base::State *_state,
                         const ::ompl::base::State *_near,
                         double distance) override;

  /// Not implemented. Throws std::domain_error
  void sampleGaussian(::ompl::base::State *_state,
                      const ::ompl::base::State *_mean, double stdDev) override;

private:
  std::unique_ptr<aikido::constraint::SampleGenerator> mGenerator;
};
}
}
#endif
