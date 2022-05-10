#include <Eigen/Dense>
#include <dart/common/Console.hpp>
#include <dart/dynamics/dynamics.hpp>

#include "aikido/common/memory.hpp"
#include "aikido/control/KinematicSimulationJointCommandExecutor.hpp"
#include "aikido/control/util.hpp"

namespace aikido {
namespace control {

//==============================================================================
extern template class JacobianExecutor<ExecutorType::VELOCITY>;

extern template class JacobianExecutor<ExecutorType::VELOCITY>;

//==============================================================================
template <ExecutorType T>
JacobianExecutor<T>::JacobianExecutor(
    ::dart::dynamics::BodyNode* eeNode,
    std::shared_ptr<JointCommandExecutor<T>> executor,
    double lambda)
  : JointCommandExecutor<T>(
      executor ? executor->getDofs() : checkNull(eeNode)->getDependentDofs(),
      executor ? executor->getTypes()
               : std::set<ExecutorType>{T, ExecutorType::STATE})
  , mEENode{checkNull(eeNode)}
  , mExecutor{executor}
  , mLambda{lambda}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
  if (!mExecutor)
  {
    this->releaseDofs();
    // Null executor, create as KinematicSimulationJointCommandExecutor
    mExecutor = std::make_shared<KinematicSimulationJointCommandExecutor<T>>(
        mEENode->getDependentDofs());
  }

  // Release sub-executor DoFs, this class owns them now
  mExecutor->releaseDofs();
}

//==============================================================================
template <ExecutorType T>
JacobianExecutor<T>::~JacobianExecutor()
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
    mExecutor->stop();
    this->stop();
  }
}

//==============================================================================
template <ExecutorType T>
std::future<int> JacobianExecutor<T>::execute(
    const Eigen::Vector6d command,
    const std::chrono::duration<double>& timeout,
    const std::chrono::system_clock::time_point& timepoint)
{

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress)
    {
      // Overwrite previous velocity command
      mCommand = Eigen::Vector6d::Zero();
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error("Command canceled.")));
    }

    mPromise.reset(new std::promise<int>());

    mCommand = command;
    mExecutionStartTime = std::chrono::system_clock::now();
    mTimeout = timeout;
    mFuture = mExecutor->execute(SE3ToJoint(mCommand), mTimeout, timepoint);
    mInProgress = true;
  }

  return mPromise->get_future();
}

//==============================================================================
template <ExecutorType T>
std::vector<double> JacobianExecutor<T>::SE3ToJoint(
    const Eigen::Vector6d command)
{

  // Command value (as Eigen)
  Eigen::Vector6d refCmd = command;
  // Return value (as Eigen)
  Eigen::VectorXd jointCmd(this->mDofs.size());
  jointCmd.setZero();

  if (refCmd.norm() != 0)
  {
    // Get Jacobian relative only to controlled joints
    Eigen::MatrixXd fullJ = mEENode->getWorldJacobian();
    Eigen::MatrixXd J(SE3_SIZE, this->mDofs.size());
    auto fullDofs = mEENode->getDependentDofs();
    for (size_t fullIndex = 0; fullIndex < fullDofs.size(); fullIndex++)
    {
      auto it = std::find(
          this->mDofs.begin(), this->mDofs.end(), fullDofs[fullIndex]);
      if (it != this->mDofs.end())
      {
        int index = it - this->mDofs.begin();
        J.col(index) = fullJ.col(fullIndex);
      }
    }

    // Calculate joint command
    Eigen::MatrixXd JtJ = J.transpose() * J;
    JtJ += mLambda * mLambda
           * Eigen::MatrixXd::Identity(JtJ.rows(), JtJ.cols());
    jointCmd = JtJ.inverse() * J.transpose() * refCmd;
  }

  return std::vector<double>(
      jointCmd.data(), jointCmd.data() + jointCmd.size());
}

//==============================================================================
template <ExecutorType T>
void JacobianExecutor<T>::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mMutex);
  mExecutor->step(timepoint);

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
      mCommand = Eigen::Vector6d::Zero();
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
    mCommand = Eigen::Vector6d::Zero();
    mExecutor->cancel();
    mInProgress = false;
    mPromise->set_value(0);
  }

  // Update underlying velocity command
  mFuture = mExecutor->execute(SE3ToJoint(mCommand), mTimeout, timepoint);
  mExecutor->step(timepoint);
}

//==============================================================================
template <ExecutorType T>
std::future<int> JacobianExecutor<T>::execute(
    const std::vector<double>& command,
    const std::chrono::duration<double>& timeout,
    const std::chrono::system_clock::time_point& timepoint)
{
  if (mInProgress)
  {
    this->cancel();
  }

  return mExecutor->execute(command, timeout, timepoint);
}

//==============================================================================
template <ExecutorType T>
void JacobianExecutor<T>::cancel()
{
  std::lock_guard<std::mutex> lock(mMutex);
  mExecutor->cancel();

  if (mInProgress)
  {
    mCommand = Eigen::Vector6d::Zero();
    mInProgress = false;
    mExecutor->cancel();
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Command canceled.")));
  }
  else
  {
    dtwarn << "[JacobianExecutor::cancel] Attempting to "
           << "cancel command, but no command in progress.\n";
  }
}

} // namespace control
} // namespace aikido
