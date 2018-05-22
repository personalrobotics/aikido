#ifndef AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_
#define AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_

#include <memory>
#include <aikido/constraint/Testable.hpp>

using aikido::constraint::DefaultTestableOutcome;
using aikido::constraint::TestableOutcome;

class PassingConstraint : public aikido::constraint::Testable
{
public:
  explicit PassingConstraint(
      std::shared_ptr<const aikido::statespace::StateSpace> stateSpace)
    : stateSpace{stateSpace}
  {
  }

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* /*state*/,
      TestableOutcome* outcome = nullptr) const override
  {
    auto defaultOutcomeObject
        = aikido::constraint::dynamic_cast_or_throw<DefaultTestableOutcome>(
            outcome);

    if (defaultOutcomeObject)
      defaultOutcomeObject->setSatisfiedFlag(true);
    return true;
  }

  std::unique_ptr<TestableOutcome> createOutcome() const override
  {
    return std::unique_ptr<TestableOutcome>(new DefaultTestableOutcome);
  }

  aikido::statespace::ConstStateSpacePtr getStateSpace() const override
  {
    return stateSpace;
  }

private:
  std::shared_ptr<const aikido::statespace::StateSpace> stateSpace;
};

class FailingConstraint : public aikido::constraint::Testable
{
public:
  explicit FailingConstraint(
      std::shared_ptr<const aikido::statespace::StateSpace> stateSpace)
    : stateSpace{stateSpace}
  {
  }

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* /*state*/,
      TestableOutcome* outcome = nullptr) const override
  {
    auto defaultOutcomeObject
        = aikido::constraint::dynamic_cast_or_throw<DefaultTestableOutcome>(
            outcome);

    if (defaultOutcomeObject)
      defaultOutcomeObject->setSatisfiedFlag(false);
    return false;
  }

  std::unique_ptr<TestableOutcome> createOutcome() const override
  {
    return std::unique_ptr<TestableOutcome>(new DefaultTestableOutcome);
  }

  aikido::statespace::ConstStateSpacePtr getStateSpace() const override
  {
    return stateSpace;
  }

private:
  std::shared_ptr<const aikido::statespace::StateSpace> stateSpace;
};

#endif // AIKIDO_TESTS_CONSTRAINT_MOCK_CONSTRAINTS_HPP_
