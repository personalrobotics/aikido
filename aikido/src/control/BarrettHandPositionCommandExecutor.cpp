#include <aikido/control/BarrettHandPositionCommandExecutor.hpp>
#include <thread>
#include <exception>
#include <stdexcept>

namespace aikido{
namespace control{

//=============================================================================
BarrettHandPositionCommandExecutor::BarrettHandPositionCommandExecutor(
  std::vector<BarrettFingerPositionCommandExecutorPtr> _positionCommandExecutors,
  std::vector<BarrettFingerSpreadCommandExecutorPtr> _spreadCommandExecutors) 
: mPositionCommandExecutors(std::move(_positionCommandExecutors))
, mSpreadCommandExecutors(std::move(_spreadCommandExecutors))
, mInExecution(false)
{
  if (mPositionCommandExecutors.size() != kNumFingers)
  {
    std::stringstream msg; 
    msg << "Need " << kNumFingers << " PositionCommandExecutors;"
    << " got " << mPositionCommandExecutors.size() << " executors."; 
    throw std::invalid_argument(msg.str());
  }

  for(int i=0; i < kNumFingers; ++i)
  {
    if (!mPositionCommandExecutors[i])
    {
      std::stringstream msg;
      msg << i << "th PositionCommandExecutor null.";
      throw std::invalid_argument(msg.str());
    }
  }

  if (mSpreadCommandExecutors.size() != kNumSpreadFingers)
  {
    std::stringstream msg; 
    msg << "Need " << kNumSpreadFingers << " SpreadCommandExecutors."
    << " got " << mSpreadCommandExecutors.size() << " executors."; 
    throw std::invalid_argument(msg.str());
  }

  for(int i=0; i < kNumSpreadFingers; ++i)
  {
    if (!mSpreadCommandExecutors[i])
    {
      std::stringstream msg;
      msg << i << "th SpreadCommandExecutor is null.";
      throw std::invalid_argument(msg.str());
    }
  }

  
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
  mPrimalGoalPositions = _positions.topRows(3); 
  mSpreadGoalPosition = _positions(3);
  mCollideWith = _collideWith;
  mInExecution = true;
  mFingerFutures.clear();

  // mFingerFutures.reserve(kNumFingers + kNumSpreadFingers);
  for(int i=0; i < kNumFingers; ++i)
    mFingerFutures.push_back(std::move(
      mPositionCommandExecutors[i]->execute(
        mPrimalGoalPositions(i), mCollideWith)));
  
  for(int i=0; i < kNumSpreadFingers; ++i)
    mFingerFutures.push_back(std::move(
      mSpreadCommandExecutors[i]->execute(
        mSpreadGoalPosition, mCollideWith)));

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

  for(int i = 0; i < mFingerFutures.size(); ++i)
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
    for(int i = 0; i < mFingerFutures.size(); ++i)
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
  for(int i=0; i < kNumFingers; ++i)
    mPositionCommandExecutors[i]->step(_timeSincePreviousCall);

  for(int i=0; i < kNumSpreadFingers; ++i)
    mSpreadCommandExecutors[i]->step(_timeSincePreviousCall);

}

}
}
