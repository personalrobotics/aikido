#ifndef AIKIDO_CONSTRAINT_NONCOLLIDING_HPP_
#define AIKIDO_CONSTRAINT_NONCOLLIDING_HPP_

#include "TestableOutcome.hpp"

namespace aikido {
namespace constraint {

class NonCollidingOutcome : public TestableOutcome {
public:
  bool is_satisfied() const;
  std::string to_string() const;
};

} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_NONCOLLIDING_HPP_
