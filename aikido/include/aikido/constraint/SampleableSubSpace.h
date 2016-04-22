#ifndef AIKIDO_CONSTRAINT_SAMPLEABLESUBSPACE_HPP_
#define AIKIDO_CONSTRAINT_SAMPLEABLESUBSPACE_HPP_
#include <vector>
#include "Sampleable.hpp"
#include "../statespace/CompoundStateSpace.hpp"

namespace aikido {
namespace constraint {

/// Sampleable for CompoundStates. 
/// It takes in a set of Sampleables and its SampleGenerators
/// sample i-th substate from the i-th Sampleable.
class SampleableSubSpace : public SampleableConstraint
{
public:

  /// Constructor.
  /// \param _stateSpace StateSpace in which this constraint operates.
  /// \param _constraints Set of sampleables. The size of _constraints
  ///        should match the number of subspaces in _stateSpace.
  ///        i-th constraint applies to i-th subspace.
  SampleableSubSpace(
    std::shared_ptr<statespace::CompoundStateSpace> _stateSpace,
    std::vector<std::shared_ptr<SampleableConstraint>> _constraints);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::CompoundStateSpace> mStateSpace;
  std::vector<std::shared_ptr<SampleableConstraint>> mConstraints;
};

} // namespace constraint
} // namespace aikido

#endif // define AIKIDO_CONSTRAINT_SAMPLEABLESUBSPACE_HPP_
