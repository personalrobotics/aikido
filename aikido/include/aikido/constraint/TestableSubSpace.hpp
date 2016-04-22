#ifndef AIKIDO_CONSTRAINT_TESTABLESUBSPACE_HPP_
#define AIKIDO_CONSTRAINT_TESTABLESUBSPACE_HPP_
#include <vector>
#include "TestableConstraint.hpp"
#include "../statespace/CompoundStateSpace.hpp"

namespace aikido {
namespace constraint {

/// Testable for CompoundStates. 
/// It takes in a set of Testables and test i-th substate on 
/// the i-th Testable.
class TestableSubSpace : public TestableConstraint
{
public:

  /// Constructor.
  /// \param _stateSpace StateSpace in which this constraint operates.
  /// \param _constraints Set of testables. The size of _constraints
  ///        should match the number of subspaces in _stateSpace.
  ///        i-th constraint applies to i-th subspace.
  TestableSubSpace(
      std::shared_ptr<statespace::CompoundStateSpace> _stateSpace,
      std::vector<TestableConstraintPtr> _constraints);

  statespace::StateSpacePtr getStateSpace() const override;

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* _state) const override;

private:
  std::shared_ptr<statespace::CompoundStateSpace> mStateSpace;
  std::vector<TestableConstraintPtr> mConstraints;
};

}  // namespace constraint
}  // namespace aikido

#endif  // define AIKIDO_CONSTRAINT_SAMPLEABLESUBSPACE_HPP_
