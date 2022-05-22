namespace aikido {
namespace robot {

//==============================================================================
template <aikido::control::ExecutorType T>
std::future<int> Robot::executeJointCommand(
    const std::vector<double>& command,
    const std::chrono::duration<double>& timeout)
{
  // Retrieve active executor
  if (mActiveExecutor.empty())
  {
    return util::make_exceptional_future<int>(
        "executeJointCommand: No active executor");
  }

  std::shared_ptr<aikido::control::JointCommandExecutor<T>> executor;
  executor
      = std::dynamic_pointer_cast<aikido::control::JointCommandExecutor<T>>(
          mExecutors[mActiveExecutor]);
  if (!executor)
  {
    return util::make_exceptional_future<int>(
        "executeJointCommand: Active executor not correct type.");
  }

  return executor->execute(command, timeout);
}

//==============================================================================
extern template std::future<int>
Robot::executeJointCommand<aikido::control::ExecutorType::POSITION>(
    const std::vector<double>& command,
    const std::chrono::duration<double>& timeout);

extern template std::future<int>
Robot::executeJointCommand<aikido::control::ExecutorType::VELOCITY>(
    const std::vector<double>& command,
    const std::chrono::duration<double>& timeout);

extern template std::future<int>
Robot::executeJointCommand<aikido::control::ExecutorType::EFFORT>(
    const std::vector<double>& command,
    const std::chrono::duration<double>& timeout);

} // namespace robot
} // namespace aikido
