#ifndef AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_
#define AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_

#include <memory>
#include <aikido/constraint/Testable.hpp>

using aikido::constraint::DefaultOutcome;
using aikido::constraint::TestableOutcome;

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
      TestableOutcome* outcome = nullptr) const override
  {
    DefaultOutcome* defaultOutcomeObject = nullptr;
    if (outcome)
    {
      defaultOutcomeObject = dynamic_cast<DefaultOutcome*>(outcome);
      if (!defaultOutcomeObject)
        throw std::invalid_argument(
            "TestableOutcome pointer is not of type DefaultOutcome.");
    }

    if (defaultOutcomeObject)
      defaultOutcomeObject->setSatisfiedFlag(true);
    return true;
  }

  std::unique_ptr<TestableOutcome> createOutcome() const
  {
    return std::unique_ptr<TestableOutcome>(new DefaultOutcome());
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
      TestableOutcome* outcome = nullptr) const override
  {
    DefaultOutcome* defaultOutcomeObject = nullptr;
    if (outcome)
    {
      defaultOutcomeObject = dynamic_cast<DefaultOutcome*>(outcome);
      if (!defaultOutcomeObject)
        throw std::invalid_argument(
            "TestableOutcome pointer is not of type DefaultOutcome.");
    }

    if (defaultOutcomeObject)
      defaultOutcomeObject->setSatisfiedFlag(false);
    return false;
  }

  std::unique_ptr<TestableOutcome> createOutcome() const
  {
    return std::unique_ptr<TestableOutcome>(new DefaultOutcome());
  }

  std::shared_ptr<aikido::statespace::StateSpace> getStateSpace() const override
  {
    return stateSpace;
  }

private:
  std::shared_ptr<aikido::statespace::StateSpace> stateSpace;
};

#endif // AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_
