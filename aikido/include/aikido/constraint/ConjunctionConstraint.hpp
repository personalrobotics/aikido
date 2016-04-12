#ifndef AIKIDO_CONSTRAINT_CONJUNCTION_CONSTRAINT_HPP_
#define AIKIDO_CONSTRAINT_CONJUNCTION_CONSTRAINT_HPP_

#include "TestableConstraint.hpp"
#include <memory>
#include <vector>

namespace aikido
{
namespace constraint
{
class ConjunctionConstraint : public TestableConstraint
{
public:
  /// Construct a ConjunctionConstraint on a specific StateSpace
  ConjunctionConstraint(
      std::shared_ptr<aikido::statespace::StateSpace> stateSpace,
      std::vector<std::shared_ptr<TestableConstraint>> constraints =
          std::vector<std::shared_ptr<TestableConstraint>>());

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* state) const override;

  const std::shared_ptr<aikido::statespace::StateSpace> getStateSpace()
      const override;

  /// Add a TestableConstraint to the conjunction
  /// \param constraint a constraint in the same StateSpace as the
  ///        ConjunctionConstraint was initialize with
  void addConstraint(std::shared_ptr<TestableConstraint> constraint);

private:
  std::shared_ptr<aikido::statespace::StateSpace> mStateSpace;
  std::vector<std::shared_ptr<TestableConstraint>> mConstraints;

  void testConstraintStateSpaceOrThrow(
      std::shared_ptr<TestableConstraint> constraint);
};
}  // constraint
}  // aikido

#endif  // AIKIDO_CONSTRAINT_CONJUNCTION_CONSTRAINT_HPP_
