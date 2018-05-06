#ifndef AIKIDO_CONSTRAINT_SEQUENTIALSAMPLEABLE_HPP_
#define AIKIDO_CONSTRAINT_SEQUENTIALSAMPLEABLE_HPP_

#include "aikido/constraint/Sampleable.hpp"

namespace aikido {
namespace constraint {

/// Sampleable supporting a composition of multiple sampleables which
/// are used for sampling in sequence. The operating sampleable switches to
/// the next one in the set upon exhaustion.
class SequentialSampleable : public Sampleable
{
public:
  /// Constructor.
  /// \param[in] stateSpace StateSpace in which this constraint operates.
  /// \param[in] sampleables Set of sampleables.
  SequentialSampleable(
      statespace::StateSpacePtr stateSpace,
      std::vector<SampleablePtr>& sampleables);

  // Documentation inherited.
  // TODO (avk): const-correctness after planner API is merged.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  /// StateSpace in which the constraint operates.
  statespace::StateSpacePtr mStateSpace;

  /// Set of sampleables.
  std::vector<SampleablePtr> mSampleables;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_CYCLICSAMPLEABLE_HPP_
