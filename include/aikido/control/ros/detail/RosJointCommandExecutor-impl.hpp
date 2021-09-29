namespace aikido {
namespace control {
namespace ros {

//==============================================================================
extern template class RosJointCommandExecutor<ExecutorType::POSITION>;

extern template class RosJointCommandExecutor<ExecutorType::VELOCITY>;

extern template class RosJointCommandExecutor<ExecutorType::EFFORT>;

// Convert Dof List to Joint name List
static std::vector<std::string> jointNamesFromDofs(
    const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs)
{
  std::vector<std::string> jointNames;
  for (auto dof : dofs)
  {
    jointNames.push_back(dof->getName());
  }
  return jointNames;
}

//==============================================================================
template <ExecutorType T>
RosJointCommandExecutor<T>::RosJointCommandExecutor(
    ::ros::NodeHandle node,
    const std::string& controllerName,
    const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs,
    const std::chrono::milliseconds connectionTimeout,
    const std::chrono::milliseconds connectionPollingPeriod)
  // Does not update DoF values directly
  : aikido::control::JointCommandExecutor<T>(
      dofs, std::set<ExecutorType>{ExecutorType::READONLY})
  , mClient{
        node,
        controllerName + "/joint_group_command",
        jointNamesFromDofs(dofs),
        connectionTimeout,
        connectionPollingPeriod}
{
  // Do nothing
}

//==============================================================================
template <ExecutorType T>
RosJointCommandExecutor<T>::~RosJointCommandExecutor()
{
  this->stop();
}

//==============================================================================
template <ExecutorType T>
std::future<int> RosJointCommandExecutor<T>::execute(
    const std::vector<double>& command,
    const std::chrono::duration<double>& timeout)
{
  ::ros::Duration duration;
  // Rounds to floor(seconds)
  auto timeoutSecs = std::chrono::duration_cast<std::chrono::seconds>(timeout);
  duration.sec = timeoutSecs.count();
  duration.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      timeout - timeoutSecs)
                      .count();
  return mClient.execute(T, command, duration);
}

//==============================================================================
template <ExecutorType T>
void RosJointCommandExecutor<T>::step(
    const std::chrono::system_clock::time_point& /* timepoint */)
{
  mClient.step();
}

//==============================================================================
template <ExecutorType T>
void RosJointCommandExecutor<T>::cancel()
{
  mClient.cancel();
}

} // namespace ros
} // namespace control
} // namespace aikido
