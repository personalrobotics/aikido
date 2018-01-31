#ifndef AIKIDO_CONSTRAINT_SMARTPOINTER_HPP_
#define AIKIDO_CONSTRAINT_SMARTPOINTER_HPP_

#include "aikido/common/smart_pointer.hpp"

namespace aikido {
namespace constraint {

AIKIDO_COMMON_DECLARE_SMART_POINTERS(Differentiable)
AIKIDO_COMMON_DECLARE_SMART_POINTERS(Projectable)
AIKIDO_COMMON_DECLARE_SMART_POINTERS(Testable)
AIKIDO_COMMON_DECLARE_SMART_POINTERS(Sampleable)

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_STATESPACE_SMARTPOINTER_HPP_
