#ifndef AIKIDO_CONSTRAINT_UNIFORM_SO2UNIFORMSAMPLER_HPP_
#define AIKIDO_CONSTRAINT_UNIFORM_SO2UNIFORMSAMPLER_HPP_
#include "../../statespace/SO2.hpp"
#include "../Sampleable.hpp"

namespace aikido {
namespace constraint {

/// Uniform sampler for SO2States. Its SampleGenerators will sample
/// uniformly from SO2, and the sequence of samples is
/// deterministically generated given a random number generator seed. 
class SO2Sampleable
  : public constraint::Sampleable
{
public:
  /// Constructor.
  /// \param _space SO2 in which this constraint operates.
  /// \param _rng Random number generator which determines the sampling
  ///        sequence of this constraint's SampleGenerators.
  SO2Sampleable(
    std::shared_ptr<statespace::SO2> _space,
    std::unique_ptr<util::RNG> _rng);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::SO2> mSpace;
  std::unique_ptr<util::RNG> mRng;
};


} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_UNIFORM_SO2UNIFORMSAMPLER_HPP_
