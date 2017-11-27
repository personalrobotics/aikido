#ifndef AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_
#define AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_

#include <string>

namespace aikido {
namespace constraint {

class TestableOutcome
{
public:
  virtual bool isSatisfied() const = 0;
  virtual std::string toString() const = 0;
};

} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_
