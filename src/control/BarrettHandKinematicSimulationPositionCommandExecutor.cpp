#include <aikido/control/BarrettHandKinematicSimulationPositionCommandExecutor.hpp>
#include <thread>
#include <exception>
#include <stdexcept>

namespace aikido{
namespace control{

constexpr std::chrono::milliseconds BarrettHandKinematicSimulationPositionCommandExecutor::kWaitPeriod;

//=============================================================================
BarrettHandKinematicSimulationPositionCommandExecutor
::BarrettHandKinematicSimulationPositionCommandExecutor(
  const std::array<BarrettFingerKinematicSimulationPositionCommandExecutorPtr, 3>& positionCommandExecutors,
  BarrettFingerKinematicSimulationSpreadCommandExecutorPtr spreadCommandExecutor,
  ::dart::collision::CollisionGroupPtr collideWith)
: mPositionCommandExecutors(std::move(positionCommandExecutors))
, mSpreadCommandExecutor(std::move(spreadCommandExecutor))
, mInExecution(false)
, mCollideWith(std::move(collideWith))
{
  for(int i=0; i < kNumPositionExecutor; ++i)
  {
    if (!mPositionCommandExecutors[i])
    {
      std::stringstream msg;
      msg << i << "th PositionCommandExecutor is null.";
      throw std::invalid_argument(msg.str());
    }
  }

  if (!mSpreadCommandExecutor)
    throw std::invalid_argument("SpreadCommandExecutor is null.");

  if (!mCollideWith)
    throw std::invalid_argument("CollideWith is null.");

}

//=============================================================================
std::future<void> BarrettHandKinematicSimulationPositionCommandExecutor
  ::execute(const Eigen::VectorXd& goalPositions)
{
  std::lock_guard<std::mutex> lockSpin(mMutex);

  if (mInExecution)
    throw std::runtime_error("Another command in execution.");

  if (goalPositions.size() != 4)
  {
    std::stringstream message;
    message << "GoalPositions must have 4 elements, but ["
      << goalPositions.size() << "] given.";
    throw std::invalid_argument(message.str());
  }

  mPromise.reset(new std::promise<void>());
  mProximalGoalPositions = goalPositions.topRows(3);
  mSpreadGoalPosition = goalPositions(3);
  mInExecution = true;
  mLastExecutionTime = std::chrono::system_clock::now();
  mFingerFutures.clear();

  mFingerFutures.reserve(kNumPositionExecutor + kNumSpreadExecutor);
  for(int i=0; i < kNumPositionExecutor; ++i)
    mFingerFutures.emplace_back(
      mPositionCommandExecutors[i]->execute(
        mProximalGoalPositions(i)));

  mFingerFutures.emplace_back(
    mSpreadCommandExecutor->execute(
      mSpreadGoalPosition));

  return mPromise->get_future();

}

//=============================================================================
void BarrettHandKinematicSimulationPositionCommandExecutor::step()
{
  using namespace std::chrono;

  std::lock_guard<std::mutex> lock(mMutex);

  auto timeSincePreviousCall = system_clock::now() - mLastExecutionTime;
  mLastExecutionTime = system_clock::now();

  if (!mInExecution)
    return;

  // Check whether fingers have completed.
  std::vector<std::future_status> statuses;
  std::exception_ptr expr;
  bool allFingersCompleted = true;

  for(size_t i = 0; i < mFingerFutures.size(); ++i)
  {
    // Check the status of each finger
    auto status = mFingerFutures[i].wait_for(kWaitPeriod);
    if (status != std::future_status::ready)
    {
      allFingersCompleted = false;
      break;
    }
  }

  if (allFingersCompleted)
  {
    for(size_t i = 0; i < mFingerFutures.size(); ++i)
    {
      try{
        mFingerFutures[i].get();
      }
      catch (...){
        expr = std::current_exception();
        break;
      }
    }
  }

  // Termination condition
  if (expr || allFingersCompleted)
  {
    if (expr)
      mPromise->set_exception(expr);
    else if (allFingersCompleted)
      mPromise->set_value();

    mInExecution = false;
    return;
  }

  // Call the finger executors' step function.
  auto period = duration_cast<milliseconds>(timeSincePreviousCall);
  for(int i=0; i < kNumPositionExecutor; ++i)
    mPositionCommandExecutors[i]->step(period);

  mSpreadCommandExecutor->step(period);

}

//=============================================================================
bool BarrettHandKinematicSimulationPositionCommandExecutor::setCollideWith(
  ::dart::collision::CollisionGroupPtr collideWith)
{
  std::lock_guard<std::mutex> lockSpin(mMutex);

  if (mInExecution)
    return false;

  mCollideWith = collideWith;
  return true;
}

} // control
} // aikido
