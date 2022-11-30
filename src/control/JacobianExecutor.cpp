#include "aikido/control/JacobianExecutor.hpp"

namespace aikido {
namespace control {

template class JacobianExecutor<ExecutorType::VELOCITY>;

template class JacobianExecutor<ExecutorType::EFFORT>;

} // namespace control
} // namespace aikido
