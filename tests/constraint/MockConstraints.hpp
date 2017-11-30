#ifndef AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_
#define AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_

#include <memory>
#include <aikido/constraint/Testable.hpp>

class PassingConstraint : public aikido::constraint::Testable
{
public:
  explicit PassingConstraint(
      std::shared_ptr<aikido::statespace::StateSpace> stateSpace)
    : stateSpace{stateSpace}
  {
  }

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* /*state*/,
      aikido::constraint::TestableOutcome* /*outcome*/ = nullptr) const override
  {
    return true;
  }

  std::shared_ptr<aikido::statespace::StateSpace> getStateSpace() const override
  {
    return stateSpace;
  }

private:
  std::shared_ptr<aikido::statespace::StateSpace> stateSpace;
};

class FailingConstraint : public aikido::constraint::Testable
{
public:
  explicit FailingConstraint(
      std::shared_ptr<aikido::statespace::StateSpace> stateSpace)
    : stateSpace{stateSpace}
  {
  }

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* /*state*/,
      aikido::constraint::TestableOutcome* /*outcome*/ = nullptr) const override
  {
    return false;
  }

  std::shared_ptr<aikido::statespace::StateSpace> getStateSpace() const override
  {
    return stateSpace;
  }

private:
  std::shared_ptr<aikido::statespace::StateSpace> stateSpace;
};

#endif // AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_
