#ifndef AIKIDO_CONSTRAINT_CYCLICSAMPLEABLE_HPP_
#define AIKIDO_CONSTRAINT_CYCLICSAMPLEABLE_HPP_

#include "aikido/constraint/Sampleable.hpp"

namespace aikido {
namespace constraint {

/// Constraint that turns a finite sampleable constraint into
/// a cyclic sampleable constraint.
/// It's generator  will generate samples in the same order as
/// the original sampleable constraint, but once the samples are exhausted,
/// it will cycle through the samples, starting from the initial sample.
/// The original sampleable should be finite.
class CyclicSampleable : public Sampleable
{
public:
  /// Constructor.
  /// \param _sampleable Sampleable whose samples are to be iterated.
  explicit CyclicSampleable(SampleablePtr _sampleable);

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  SampleablePtr mSampleable;
  statespace::ConstStateSpacePtr mStateSpace;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_CYCLICSAMPLEABLE_HPP_
