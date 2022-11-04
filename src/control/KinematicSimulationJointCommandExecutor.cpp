#include "aikido/control/KinematicSimulationJointCommandExecutor.hpp"

namespace aikido {
namespace control {

template class KinematicSimulationJointCommandExecutor<ExecutorType::POSITION>;

template class KinematicSimulationJointCommandExecutor<ExecutorType::VELOCITY>;

template class KinematicSimulationJointCommandExecutor<ExecutorType::EFFORT>;

} // namespace control
} // namespace aikido
