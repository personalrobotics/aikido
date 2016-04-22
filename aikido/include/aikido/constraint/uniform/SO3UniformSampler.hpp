#ifndef AIKIDO_CONSTRAINT_UNIFORM_SO3UNIFORMSAMPLER_HPP_
#define AIKIDO_CONSTRAINT_UNIFORM_SO3UNIFORMSAMPLER_HPP_
#include "../../statespace/SO3.hpp"
#include "../Sampleable.hpp"

namespace aikido {
namespace statespace {

/// Uniform sampler for SO3States. Its SampleGenerators will sample
/// uniformly from SO3, and the sequence of samples is
/// deterministically generated given a random number generator seed. 
class SO3UniformSampler : public constraint::SampleableConstraint
{
public:

  /// Constructor.
  /// \param _space SO3 in which this constraint operates.
  /// \param _rng Random number generator which determines the sampling
  ///        sequence of this constraint's SampleGenerators.
  SO3UniformSampler(
    std::shared_ptr<statespace::SO3> _space,
    std::unique_ptr<util::RNG> _rng);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::SO3> mSpace;
  std::unique_ptr<util::RNG> mRng;
};

} // namespace statespace
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_UNIFORM_SO3UNIFORMSAMPLER_HPP_
