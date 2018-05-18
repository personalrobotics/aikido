#ifndef AIKIDO_CONSTRAINT_SEQUENTIALSAMPLEABLE_HPP_
#define AIKIDO_CONSTRAINT_SEQUENTIALSAMPLEABLE_HPP_

#include "aikido/constraint/Sampleable.hpp"

namespace aikido {
namespace constraint {

/// Sampleable that wraps a sequence of Sampleables. Exhausts the current
/// Sampleable before sampling from the next Sampleable in the sequence.
class SequentialSampleable : public Sampleable
{
public:
  /// Constructor.
  /// \param[in] stateSpace StateSpace in which this constraint operates.
  /// \param[in] sampleables Sequence of sampleables.
  SequentialSampleable(
      statespace::StateSpacePtr stateSpace,
      const std::vector<ConstSampleablePtr>& sampleables);

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  /// StateSpace in which the constraint operates.
  statespace::ConstStateSpacePtr mStateSpace;

  /// Sequence of sampleables.
  const std::vector<ConstSampleablePtr> mSampleables;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_CYCLICSAMPLEABLE_HPP_
