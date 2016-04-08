#ifndef AIKIDO_CONSTRAINT_TESTABLE_CONSTRAINT_HPP_
#define AIKIDO_CONSTRAINT_TESTABLE_CONSTRAINT_HPP_

#include <memory>
#include "../statespace/StateSpace.hpp"

using aikido::statespace::StateSpace;

namespace aikido
{
namespace constraint
{
class TestableConstraint
{
public:
  virtual ~TestableConstraint() = default;
  virtual bool isSatisfied(const StateSpace::State*) const;
  virtual const std::shared_ptr<StateSpace> getStateSpace() const;
};

}  // constraint
}  // aikido
#endif  // AIKIDO_CONSTRAINT_TESTABLE_CONSTRAINT_HPP_
