#ifndef AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_
#define AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_

namespace aikido {
namespace constraint {

class TestableOutcome {
public:
  virtual bool is_satisfied() const = 0;
  virtual std::string to_string() const = 0;
};

} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_
