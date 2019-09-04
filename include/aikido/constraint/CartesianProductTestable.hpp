#ifndef AIKIDO_CONSTRAINT_TESTABLESUBSPACE_HPP_
#define AIKIDO_CONSTRAINT_TESTABLESUBSPACE_HPP_

#include <vector>

#include "aikido/constraint/Testable.hpp"
#include "aikido/statespace/CartesianProduct.hpp"

namespace aikido {
namespace constraint {

/// Testable for CompoundStates.
/// It takes in a set of Testables and test i-th substate on
/// the i-th Testable.
class CartesianProductTestable : public Testable
{
public:
  /// Constructor.
  /// \param _stateSpace StateSpace in which this constraint operates.
  /// \param _constraints Set of testables. The size of _constraints
  ///        should match the number of subspaces in _stateSpace.
  ///        i-th constraint applies to i-th subspace.
  CartesianProductTestable(
      std::shared_ptr<const statespace::CartesianProduct> _stateSpace,
      std::vector<ConstTestablePtr> _constraints);

  statespace::ConstStateSpacePtr getStateSpace() const override;

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* _state,
      TestableOutcome* outcome = nullptr) const override;

  /// Return an instance of DefaultTestableOutcome, since this class doesn't
  /// have a more specialized TestableOutcome derivative assigned to it.
  std::unique_ptr<TestableOutcome> createOutcome() const override;

private:
  std::shared_ptr<const statespace::CartesianProduct> mStateSpace;
  std::vector<ConstTestablePtr> mConstraints;
};

} // namespace constraint
} // namespace aikido

#endif // define AIKIDO_CONSTRAINT_SAMPLEABLESUBSPACE_HPP_
