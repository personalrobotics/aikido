#ifndef AIKIDO_CONSTRAINT_TESTABLE_CONSTRAINT_HPP_
#define AIKIDO_CONSTRAINT_TESTABLE_CONSTRAINT_HPP_

#include <memory>
#include "../statespace/StateSpace.hpp"

namespace aikido
{
namespace constraint
{
class TestableConstraint
{
public:
  virtual ~TestableConstraint() = default;
  virtual bool isSatisfied(const aikido::statespace::StateSpace::State* state) const = 0;
  virtual std::shared_ptr<aikido::statespace::StateSpace> getStateSpace() const = 0;
};

}  // constraint
}  // aikido
#endif  // AIKIDO_CONSTRAINT_TESTABLE_CONSTRAINT_HPP_
