#include <dart/common/Console.hpp>

#include "aikido/common/memory.hpp"

namespace aikido {
namespace control {

//==============================================================================
extern template class KinematicSimulationJointCommandExecutor<
    ExecutorType::POSITION>;

extern template class KinematicSimulationJointCommandExecutor<
    ExecutorType::VELOCITY>;

//==============================================================================
template <ExecutorType T>
KinematicSimulationJointCommandExecutor<T>::
    KinematicSimulationJointCommandExecutor(
        ::dart::dynamics::MetaSkeletonPtr metaskeleton)
  : JointCommandExecutor<T>(
      checkNull(metaskeleton)->getDofs(),
      std::set<ExecutorType>{ExecutorType::STATE})
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
}

//==============================================================================
template <ExecutorType T>
KinematicSimulationJointCommandExecutor<
    T>::~KinematicSimulationJointCommandExecutor()
{
  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress)
    {
      mInProgress = false;
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error("Command canceled.")));
    }
    this->stop();
  }
}

//==============================================================================
template <ExecutorType T>
std::future<int> KinematicSimulationJointCommandExecutor<T>::execute(
<<<<<<< HEAD
    const std::vector<double> command,
    const std::chrono::duration<double>& timeout)
{

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning
    auto promise = std::promise<int>();

    if (command.size() != this->mJoints.size())
    {
      promise.set_exception(std::make_exception_ptr(
          std::runtime_error("DOF of command does not match DOF of joints.")));
      return promise.get_future();
    }

    if (mInProgress)
    {
      // Overwrite previous command
      mCommand.clear();
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error("Command canceled.")));
    }

    mPromise.reset(new std::promise<int>());

    mCommand = command;
    mInProgress = true;
    mExecutionStartTime = std::chrono::system_clock::now();
    mStartPosition.clear();
    mTimeout = timeout;
    switch (T)
    {
      case ExecutorType::VELOCITY:

        for (size_t i = 0; i < mSkeleton->getDofs().size(); i++)
        {
          auto dof = mSkeleton->getDof(i);
          dof->setVelocity(command[i]);
          mStartPosition.push_back(dof->getPosition());
        }

        break;

      case ExecutorType::POSITION:

        for (size_t i = 0; i < mSkeleton->getDofs().size(); i++)
        {
          auto dof = mSkeleton->getDof(i);
          mStartPosition.push_back(dof->getPosition());
          if (mTimeout.count() > 1E-8)
          {
            dof->setVelocity(
                (command[i] - mStartPosition[i]) / timeout.count());
          }
        }

        break;

      default:
        // Other Executors not implemented
        mCommand.clear();
        mPromise->set_exception(std::make_exception_ptr(
            std::logic_error("Executor not implemented")));
        mInProgress = false;
    }
=======
    const std::vector<double>& command,
    const std::chrono::duration<double>& timeout)
{
  auto promise = std::promise<int>();

  if (command.size() != this->mDofs.size())
  {
    promise.set_exception(std::make_exception_ptr(
        std::runtime_error("DOF of command does not match DOF of joints.")));
    return promise.get_future();
  }

  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning
  if (mInProgress)
  {
    // Overwrite previous command
    mCommand.clear();
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Command canceled.")));
  }

  mPromise.reset(new std::promise<int>());

  mCommand = command;
  mInProgress = true;
  mExecutionStartTime = std::chrono::system_clock::now();
  mTimeout = timeout;

  // Set start positions
  mStartPosition.clear();
  for (std::size_t i = 0; i < this->mDofs.size(); ++i)
  {
    auto dof = this->mDofs[i];
    mStartPosition.push_back(dof->getPosition());
  }
  switch (T)
  {
    case ExecutorType::VELOCITY:

      for (std::size_t i = 0; i < this->mDofs.size(); ++i)
      {
        auto dof = this->mDofs[i];
        dof->setVelocity(command[i]);
      }

      break;

    case ExecutorType::POSITION:

      for (std::size_t i = 0; i < this->mDofs.size(); ++i)
      {
        auto dof = this->mDofs[i];
        if (mTimeout.count() > 0.0)
        {
          dof->setVelocity((command[i] - mStartPosition[i]) / timeout.count());
        }
      }

      break;

    default:
      // Other Executors not implemented
      mCommand.clear();
      mPromise->set_exception(std::make_exception_ptr(
          std::logic_error("Executor not implemented")));
      mInProgress = false;
>>>>>>> master
  }

  return mPromise->get_future();
}

//==============================================================================
template <ExecutorType T>
void KinematicSimulationJointCommandExecutor<T>::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInProgress)
    return;

  const auto timeSinceBeginning = timepoint - mExecutionStartTime;
  const auto executionTime
      = std::chrono::duration<double>(timeSinceBeginning).count();

  // executionTime may be negative if the thread calling \c step is queued
  // before and dequeued after \c execute is called.
  if (executionTime < 0)
    return;

  // Set joint states
<<<<<<< HEAD
  double interpTime;
  switch (T)
  {
    case ExecutorType::VELOCITY:
      for (size_t i = 0; i < mSkeleton->getDofs().size(); i++)
      {
        auto dof = mSkeleton->getDof(i);
=======
  switch (T)
  {
    case ExecutorType::VELOCITY: {
      for (size_t i = 0; i < this->mDofs.size(); ++i)
      {
        auto dof = this->mDofs[i];
>>>>>>> master
        dof->setPosition(mStartPosition[i] + executionTime * mCommand[i]);
      }

      break;
<<<<<<< HEAD

    case ExecutorType::POSITION:
      // Instantaneous movement if no timeout.
      interpTime
=======
    }
    case ExecutorType::POSITION: {
      // Instantaneous movement if no timeout.
      double interpTime
>>>>>>> master
          = (mTimeout.count() == 0) ? 1.0 : executionTime / mTimeout.count();
      // Stop at the correct position
      if (interpTime > 1.0)
      {
        interpTime = 1.0;
      }

<<<<<<< HEAD
      for (size_t i = 0; i < mSkeleton->getDofs().size(); i++)
      {
        auto dof = mSkeleton->getDof(i);
=======
      for (size_t i = 0; i < this->mDofs.size(); ++i)
      {
        auto dof = this->mDofs[i];
>>>>>>> master
        double pose
            = (1.0 - interpTime) * mStartPosition[i] + interpTime * mCommand[i];
        dof->setPosition(pose);
      }

      break;
<<<<<<< HEAD

    default:
      // SHOULD NEVER REACH
=======
    }
    default:
>>>>>>> master
      throw std::logic_error(
          "Other KinematicSimulationExecutors not implemented.");
  }

  // Check if command has timed out
  if (mTimeout.count() > 0.0 && executionTime >= mTimeout.count())
  {
    mCommand.clear();
    mInProgress = false;
    mPromise->set_value(0);
  }
}

//==============================================================================
template <ExecutorType T>
void KinematicSimulationJointCommandExecutor<T>::cancel()
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (mInProgress)
  {
    mCommand.clear();
    mInProgress = false;
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Command canceled.")));
  }
  else
  {
    dtwarn << "[KinematicSimulationJointCommandExecutor::cancel] Attempting to "
           << "cancel command, but no command in progress.\n";
  }
}

} // namespace control
} // namespace aikido
