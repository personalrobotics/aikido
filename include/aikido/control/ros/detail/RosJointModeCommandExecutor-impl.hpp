namespace aikido {
namespace control {
namespace ros {

//==============================================================================
// Convert Dof List to Joint name List
static std::vector<std::string> jointNamesFromDofs(
    const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs) //Rajat TODO: Put this in a util file as it is shared?
{
  std::vector<std::string> jointNames;
  for (const auto& dof : dofs)
  {
    jointNames.push_back(dof->getName());
  }
  return jointNames;
}

//==============================================================================
RosJointModeCommandExecutor::RosJointModeCommandExecutor(
    ::ros::NodeHandle node,
    const std::string& controllerName,
    const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs,
    const std::chrono::milliseconds connectionTimeout,
    const std::chrono::milliseconds connectionPollingPeriod)
  // Does not update DoF values directly
  : aikido::control::JointModeCommandExecutor(
      dofs, std::set<ExecutorType>{ExecutorType::READONLY}) //Rajat TODO: Do we require READONLY ?
  , mClient{
        node,
        controllerName + "/joint_mode_command",
        jointNamesFromDofs(dofs),
        connectionTimeout,
        connectionPollingPeriod}
{
  // Do nothing
}

//==============================================================================
RosJointModeCommandExecutor::~RosJointModeCommandExecutor()
{
  this->stop();
}

//==============================================================================
std::future<int> RosJointModeCommandExecutor::execute(
    const std::vector<hardware_interface::JointCommandModes>& command,
    const std::chrono::duration<double>& timeout,
    const std::chrono::system_clock::time_point& /* timepoint */)
{
  ::ros::Duration duration;
  // Rounds to floor(seconds)
  auto timeoutSecs = std::chrono::duration_cast<std::chrono::seconds>(timeout);
  duration.sec = timeoutSecs.count();
  duration.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      timeout - timeoutSecs)
                      .count();
  return mClient.execute(command, duration);
}

//==============================================================================
void RosJointModeCommandExecutor::step(
    const std::chrono::system_clock::time_point& /* timepoint */)
{
  mClient.step();
}

//==============================================================================
void RosJointModeCommandExecutor::cancel()
{
  mClient.cancel();
}

} // namespace ros
} // namespace control
} // namespace aikido
