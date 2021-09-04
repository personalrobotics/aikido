#include "aikido/control/KinematicSimulationJointCommandExecutor.hpp"

namespace aikido {
namespace control {

template class KinematicSimulationJointCommandExecutor<ExecutorType::POSITION>;

template class KinematicSimulationJointCommandExecutor<ExecutorType::VELOCITY>;

} // namespace control
} // namespace aikido