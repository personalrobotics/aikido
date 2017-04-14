#include <aikido/control/BarrettHandPositionCommandExecutor.hpp>
#include <thread>
#include <exception>
#include <stdexcept>

namespace aikido{
namespace control{

constexpr std::chrono::milliseconds BarrettHandPositionCommandExecutor::kWaitPeriod;

//=============================================================================
BarrettHandPositionCommandExecutor::BarrettHandPositionCommandExecutor(
  std::array<BarrettFingerPositionCommandExecutorPtr, 3> _positionCommandExecutors,
  BarrettFingerSpreadCommandExecutorPtr _spreadCommandExecutor) 
: mPositionCommandExecutors(std::move(_positionCommandExecutors))
, mSpreadCommandExecutor(std::move(_spreadCommandExecutor))
, mInExecution(false)
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
}

//=============================================================================
std::future<void> BarrettHandPositionCommandExecutor::execute(
  Eigen::Matrix<double, 4, 1> _positions,
  ::dart::collision::CollisionGroupPtr _collideWith)
{
  if (!_collideWith)
    throw std::invalid_argument("CollideWith is null.");

  std::lock_guard<std::mutex> lockSpin(mMutex);
  
  if (mInExecution)
    throw std::runtime_error("Another command in execution.");

  mPromise.reset(new std::promise<void>());
  mProximalGoalPositions = _positions.topRows(3); 
  mSpreadGoalPosition = _positions(3);
  mCollideWith = _collideWith;
  mInExecution = true;
  mFingerFutures.clear();

  mFingerFutures.reserve(kNumPositionExecutor + kNumSpreadExecutor);
  for(int i=0; i < kNumPositionExecutor; ++i)
    mFingerFutures.emplace_back(
      mPositionCommandExecutors[i]->execute(
        mProximalGoalPositions(i), mCollideWith));
  
  mFingerFutures.emplace_back(
    mSpreadCommandExecutor->execute(
      mSpreadGoalPosition, mCollideWith));

  return mPromise->get_future();

}

//=============================================================================
void BarrettHandPositionCommandExecutor::step(double _timeSincePreviousCall)
{
  using std::chrono::milliseconds; 

  std::lock_guard<std::mutex> lock(mMutex);

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
  for(int i=0; i < kNumPositionExecutor; ++i)
    mPositionCommandExecutors[i]->step(_timeSincePreviousCall);

  mSpreadCommandExecutor->step(_timeSincePreviousCall);

}

}
}
