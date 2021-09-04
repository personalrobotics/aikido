namespace aikido {
namespace control {
namespace ros {

//==============================================================================
extern template class RosJointCommandExecutor<ExecutorType::POSITION>;

extern template class RosJointCommandExecutor<ExecutorType::VELOCITY>;

extern template class RosJointCommandExecutor<ExecutorType::EFFORT>;

//==============================================================================
template<ExecutorType T>
RosJointCommandExecutor<T>::RosJointCommandExecutor(
    ::ros::NodeHandle node,
    const std::string& controllerName,
    std::vector<std::string> jointNames,
    const std::chrono::milliseconds connectionTimeout,
    const std::chrono::milliseconds connectionPollingPeriod)
  : aikido::control::JointCommandExecutor<T>(jointNames)
  , mClient{node,
            controllerName + "/joint_group_command",
            jointNames,
            connectionTimeout,
            connectionPollingPeriod}
{
  // Do nothing.
}

//==============================================================================
template<ExecutorType T>
RosJointCommandExecutor<T>::~RosJointCommandExecutor()
{
  this->stop();
}

//==============================================================================
template<ExecutorType T>
std::future<int> RosJointCommandExecutor<T>::execute(
    const std::vector<double> command,
    const std::chrono::duration<double>& timeout)
{
  ::ros::Duration duration;
  duration.sec
      = std::chrono::duration_cast<std::chrono::seconds>(timeout).count();
  duration.nsec
      = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count()
        % 1000000000UL;
  return mClient.execute(T, command, duration);
}

//==============================================================================
template<ExecutorType T>
void RosJointCommandExecutor<T>::step(
    const std::chrono::system_clock::time_point& /* timepoint */)
{
  mClient.step();
}

//==============================================================================
template<ExecutorType T>
void RosJointCommandExecutor<T>::cancel()
{
  mClient.cancel();
}

} // namespace ros
} // namespace control
} // namespace aikido
