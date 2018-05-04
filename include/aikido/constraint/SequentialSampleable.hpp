#ifndef AIKIDO_CONSTRAINT_SEQUENTIALSAMPLEABLE_HPP_
#define AIKIDO_CONSTRAINT_SEQUENTIALSAMPLEABLE_HPP_

#include "Sampleable.hpp"

namespace aikido {
namespace constraint {

/// Constraint that turns a finite sampleable constraint into
/// a cyclic sampleable constraint.
/// It's generator  will generate samples in the same order as
/// the original sampleable constraint, but once the samples are exhausted,
/// it will cycle through the samples, starting from the initial sample.
/// The original sampleable should be finite.
class SequentialSampleable : public Sampleable
{
public:
  /// Constructor.
  /// \param _sampleable Sampleable whose samples are to be iterated.
  SequentialSampleable(statespace::StateSpacePtr stateSpace,
                       std::vector<SampleablePtr> sampleable);

  // Documentation inherited.
  // TODO (avk): const-correctness after planner API is merged.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  statespace::StateSpacePtr mStateSpace;
  std::vector<SampleablePtr> mSampleables;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_CYCLICSAMPLEABLE_HPP_
