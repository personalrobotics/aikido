#ifndef AIKIDO_CONSTRAINT_TESTABLESUBSPACE_H_
#define AIKIDO_CONSTRAINT_TESTABLESUBSPACE_H_
#include <vector>
#include "TestableConstraint.hpp"
#include "../statespace/CompoundStateSpace.hpp"

namespace aikido
{
namespace constraint
{
class TestableSubSpace : public TestableConstraint
{
public:
  TestableSubSpace(
      std::shared_ptr<statespace::CompoundStateSpace> _stateSpace,
      std::vector<std::shared_ptr<TestableConstraint>> _constraints);

  statespace::StateSpacePtr getStateSpace() const override;

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* state) const override;

private:
  std::shared_ptr<statespace::CompoundStateSpace> mStateSpace;
  std::vector<std::shared_ptr<TestableConstraint>> mConstraints;
};

}  // namespace constraint
}  // namespace aikido

#endif  // define AIKIDO_CONSTRAINT_SAMPLEABLESUBSPACE_H_
