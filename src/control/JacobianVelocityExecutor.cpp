#include "aikido/control/JacobianVelocityExecutor.hpp"

#include <Eigen/Dense>
#include <dart/common/Console.hpp>
#include <dart/dynamics/dynamics.hpp>

#include "aikido/common/memory.hpp"

#define SE3_SIZE 6

namespace aikido {
namespace control {

static std::vector<std::string> checkNull(VelocityExecutor* executor)
{
  if (!executor)
    throw std::invalid_argument("VelocityExecutor is null.");

  return executor->getJoints();
}

//==============================================================================
JacobianVelocityExecutor::JacobianVelocityExecutor(
    ::dart::dynamics::SkeletonPtr skeleton,
    std::string eeName,
    std::shared_ptr<VelocityExecutor> executor)
  : Executor(ExecutorType::VELOCITY, checkNull(executor.get()))
  , mSkeleton{std::move(skeleton)}
  , mEEName{eeName}
  , mExecutor{std::move(executor)}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
  if (!mSkeleton)
  {
    throw std::invalid_argument("Skeleton is null.");
  }

  if (executor->getJoints() != skeletonToJointNames(mSkeleton))
  {
    throw std::invalid_argument(
        "Executor DOF doesn't match provided Skeleton DOF.");
  }

  if (!mSkeleton->getBodyNode(mEEName))
  {
    throw std::invalid_argument(
        "eeName not present as BodyNode in provided Skeleton.");
  }
}

//==============================================================================
JacobianVelocityExecutor::~JacobianVelocityExecutor()
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
    stop();
  }
}

//==============================================================================
std::future<int> JacobianVelocityExecutor::execute(
    const std::vector<double> command,
    const std::chrono::duration<double>& timeout)
{

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning
    auto promise = std::promise<int>();

    if (command.size() != SE3_SIZE)
    {
      promise.set_exception(std::make_exception_ptr(
          std::runtime_error("Command is not in SE3 (6d)")));
      return promise.get_future();
    }

    if (mInProgress)
    {
      // Overwrite previous velocity command
      mCommand.clear();
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error("Command canceled.")));
    }

    mPromise.reset(new std::promise<int>());

    mCommand = command;
    mExecutionStartTime = std::chrono::system_clock::now();
    mTimeout = timeout;
    mFuture = mExecutor->execute(SE3ToJoint(mCommand), mTimeout);
    mInProgress = true;
  }

  return mPromise->get_future();
}

//==============================================================================
std::vector<double> JacobianVelocityExecutor::SE3ToJoint(
    std::vector<double> cmd)
{

  // Command value (as Eigen)
  Eigen::Vector6d setVelocity(cmd.data());
  // Return value (as Eigen)
  Eigen::VectorXd jointVels(mSkeleton->getNumDofs());
  jointVels.setZero();

  if (setVelocity.norm() != 0)
  {
    Eigen::MatrixXd J = mSkeleton->getBodyNode(mEEName)->getWorldJacobian();
    Eigen::MatrixXd JtJ = J.transpose() * J;
    if (JtJ.determinant() != 0)
    {
      jointVels = JtJ.inverse() * J.transpose() * setVelocity;
    }
    else
    {
      throw std::runtime_error(
          "Error: at singularity. Cartesian control impossible.");
    }
  }

  return std::vector<double>(
      jointVels.data(), jointVels.data() + jointVels.size());
}

//==============================================================================
void JacobianVelocityExecutor::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInProgress)
    return;

  // Check that underlying velocity command hasn't errored
  auto status = mFuture.wait_for(std::chrono::milliseconds(1));
  if (status == std::future_status::ready)
  {
    bool fail = false;
    try
    {
      int result = mFuture.get();
      if (result)
      {
        fail = true;
        mPromise->set_value(result);
      }
    }
    catch (const std::exception& e)
    {
      fail = true;
      mPromise->set_exception(std::current_exception());
    }
    if (fail)
    {
      mCommand.clear();
      mExecutor->cancel();
      mInProgress = false;
      return;
    }
  }

  const auto timeSinceBeginning = timepoint - mExecutionStartTime;
  const auto executionTime
      = std::chrono::duration<double>(timeSinceBeginning).count();

  // executionTime may be negative if the thread calling \c step is queued
  // before and dequeued after \c execute is called.
  if (executionTime < 0)
    return;

  // Check if command has timed out
  if (mTimeout.count() > 0.0 && executionTime >= mTimeout.count())
  {
    mCommand.clear();
    mExecutor->cancel();
    mInProgress = false;
    mPromise->set_value(0);
  }

  // Update underlying velocity command
  mFuture = mExecutor->execute(SE3ToJoint(mCommand), mTimeout);
}

//==============================================================================
void JacobianVelocityExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (mInProgress)
  {
    mCommand.clear();
    mInProgress = false;
    mExecutor->cancel();
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Command canceled.")));
  }
  else
  {
    dtwarn << "[JacobianVelocityExecutor::cancel] Attempting to "
           << "cancel command, but no command in progress.\n";
  }
}

} // namespace control
} // namespace aikido
