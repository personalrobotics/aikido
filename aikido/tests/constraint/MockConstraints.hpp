#ifndef AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_
#define AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_

#include <memory>
#include <aikido/constraint/TestableConstraint.hpp>

class PassingConstraint : public aikido::constraint::TestableConstraint
{
public:
  explicit PassingConstraint(
        std::shared_ptr<aikido::statespace::StateSpace> stateSpace)
    : stateSpace{stateSpace}
  {
  }

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* state) const override
  {
    return true;
  }

  std::shared_ptr<aikido::statespace::StateSpace> getStateSpace()
      const override
  {
    return stateSpace;
  }

private:
  std::shared_ptr<aikido::statespace::StateSpace> stateSpace;
};

class FailingConstraint : public aikido::constraint::TestableConstraint
{
public:
  explicit FailingConstraint(
        std::shared_ptr<aikido::statespace::StateSpace> stateSpace)
    : stateSpace{stateSpace}
  {
  }

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* state) const override
  {
    return false;
  }

  std::shared_ptr<aikido::statespace::StateSpace> getStateSpace()
      const override
  {
    return stateSpace;
  }

private:
  std::shared_ptr<aikido::statespace::StateSpace> stateSpace;
};


#endif  // AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_
