#include "aikido/control/VisualServoingVelocityExecutor.hpp"

#include <Eigen/Dense>
#include <dart/common/Console.hpp>
#include <dart/dynamics/dynamics.hpp>

#include "aikido/control/util.hpp"

namespace aikido {
namespace control {

//==============================================================================
VisualServoingVelocityExecutor::VisualServoingVelocityExecutor(
    ::dart::dynamics::BodyNode* eeNode,
    std::shared_ptr<JacobianVelocityExecutor> executor,
    Properties properties)
  : Executor(
      executor
          ? executor->getTypes()
          : std::set<ExecutorType>{ExecutorType::VELOCITY, ExecutorType::STATE},
      executor ? executor->getDofs() : checkNull(eeNode)->getDependentDofs(),
      properties.mThreadPeriod)
  , mEENode{checkNull(eeNode)}
  , mExecutor{executor}
  , mPromise{nullptr}
  , mMutex{}
  , mProperties{properties}
{
  if (!mExecutor)
  {
    this->releaseDofs();
    // Null executor, create as JacobianVelocityExecutor
    mExecutor = std::make_shared<JacobianVelocityExecutor>(mEENode);
  }

  // Release sub-executor DoFs, this class owns them now
  mExecutor->releaseDofs();
}

//==============================================================================
VisualServoingVelocityExecutor::~VisualServoingVelocityExecutor()
{
  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mPerception)
    {
      mPerception = nullptr;
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error("Command canceled.")));
    }
    mExecutor->stop();
    this->stop();
  }
}

//==============================================================================
std::future<int> VisualServoingVelocityExecutor::execute(
    std::function<std::shared_ptr<Eigen::Isometry3d>(void)> perception,
    const std::chrono::system_clock::time_point& timepoint)
{

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mPerception)
    {
      // Cancel Previous Command
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error("Command canceled.")));
      mExecutor->cancel();
    }

    // Check that perception is callable
    if (!perception)
    {
      return make_exceptional_future<int>("Perception function not callable.");
    }
    mPromise.reset(new std::promise<int>());
    mPerception = perception;
    mFuture = std::future<int>(); // Invalid
    mLastPerceivedTime = timepoint;
  }

  return mPromise->get_future();
}

//==============================================================================
void VisualServoingVelocityExecutor::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mMutex);
  mExecutor->step(timepoint);

  if (!mPerception)
    return;

  // Check that underlying velocity command hasn't errored
  if (mFuture.valid())
  {
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
        mPerception = nullptr;
        mExecutor->cancel();
        return;
      }
    }
  }

  // Try Perception
  auto goal = mPerception();
  if (goal != nullptr)
  {
    mLastPerceivedTime = timepoint;
    // Construct error vector
    Eigen::Vector3d error
        = goal->translation() - mEENode->getWorldTransform().translation();

    // If we are within tolerance, exit with success
    if (error.norm() <= mProperties.mGoalTolerance)
    {
      mExecutor->cancel();
      mPerception = nullptr;
      mPromise->set_value(0); // Success code, TODO(egordon), switch to enum
      return;
    }

    // Update underlying velocity command
    Eigen::Vector6d command = Eigen::Vector6d::Zero();
    command.tail<3>() = error.normalized() * mProperties.mApproachVelocity;
    mFuture = mExecutor->execute(command, mProperties.mTimeout, timepoint);
    mExecutor->step(timepoint);
  }

  // Check if perception has timed out
  const auto timeSincePerception
      = std::chrono::duration<double>(timepoint - mLastPerceivedTime);
  if (mProperties.mTimeout.count() > 0.0
      && timeSincePerception >= mProperties.mTimeout)
  {
    mExecutor->cancel();
    mPerception = nullptr;
    mPromise->set_value(
        -1); // Perception Timeout Error code, TODO(egordon), switch to enum
    return;
  }
}

//==============================================================================
void VisualServoingVelocityExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(mMutex);
  mExecutor->cancel();

  if (mPerception)
  {
    mPerception = nullptr;
    mExecutor->cancel();
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Command canceled.")));
  }
  else
  {
    dtwarn << "[VisualServoingVelocityExecutor::cancel] Attempting to "
           << "cancel command, but no command in progress.\n";
  }
}

} // namespace control
} // namespace aikido
