#include "aikido/control/ros/RosJointCommandExecutor.hpp"

namespace aikido {
namespace control {
namespace ros {

template class RosJointCommandExecutor<ExecutorType::POSITION>;

template class RosJointCommandExecutor<ExecutorType::VELOCITY>;

template class RosJointCommandExecutor<ExecutorType::EFFORT>;

} // namespace ros
} // namespace control
} // namespace aikido