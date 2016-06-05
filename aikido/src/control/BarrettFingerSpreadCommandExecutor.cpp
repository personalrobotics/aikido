#include <aikido/control/BarrettFingerSpreadCommandExecutor.hpp>
#include <thread>

namespace aikido{
namespace control{

//=============================================================================
BarrettFingerSpreadCommandExecutor::BarrettFingerSpreadCommandExecutor(
  ::dart::dynamics::ChainPtr _finger, int _spread, 
  ::dart::collision::CollisionDetectorPtr _collisionDetector,
  ::dart::collision::Option _collisionOptions)
: mFinger(std::move(_finger))
, mCollisionDetector(std::move(_collisionDetector))
, mCollisionOptions(std::move(_collisionOptions))
, mInExecution(false)
{
  if (!mFinger)
    throw std::invalid_argument("Finger is null.");

  mSpreadDof = mFinger->getDof(_spread);
  if (!mSpreadDof)
    throw std::invalid_argument("Finger does not have spread dof.");

  if (!mCollisionDetector)
    throw std::invalid_argument("CollisionDetctor is null.");

  mSpreadCollisionGroup = mCollisionDetector->createCollisionGroup();
  for(auto body: mFinger->getBodyNodes())
    mSpreadCollisionGroup->addShapeFramesOf(body);

  mDofLimits = mSpreadDof->getPositionLimits();
}

//=============================================================================
std::future<void> BarrettFingerSpreadCommandExecutor::execute(
  double _goalPosition,
  ::dart::collision::CollisionGroupPtr _collideWith)
{
  if (!_collideWith)
    throw std::invalid_argument("CollideWith is null.");

  if (!mFinger->isAssembled())
    throw std::runtime_error("Finger no longer linked.");

  {
    std::lock_guard<std::mutex> lock(mMutex);

    if (mInExecution)
      throw std::runtime_error("Another command in execution.");

    mPromise.reset(new std::promise<void>());

    if (_goalPosition < mDofLimits.first)
      mGoalPosition = mDofLimits.first;
    else if (_goalPosition > mDofLimits.second)
      mGoalPosition = mDofLimits.second;
    else
      mGoalPosition = _goalPosition;

    mGoalPosition = _goalPosition; 
    mCollideWith = _collideWith;
    mInExecution = true;

    return mPromise->get_future();
  }
}

//=============================================================================
void BarrettFingerSpreadCommandExecutor::step(double _timeSincePreviousCall)
{
  using std::chrono::milliseconds; 
  using ::dart::collision::Result;

  // Terminate the thread if mRunning is false.
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInExecution)
    return;
  
  // Current spread
  double spread = mSpreadDof->getPosition();
  
  // Check collision 
  Result collisionResult;
  bool collision = mCollisionDetector->collide(
    mSpreadCollisionGroup.get(), mCollideWith.get(),
    mCollisionOptions, collisionResult);

  // Termination condition
  if (collision || std::abs(spread - mGoalPosition) < kTolerance)
  {
    mPromise->set_value();
    mInExecution = false;
    return;
  }
  
  // Move spread
  double newSpread;
  if (spread > mGoalPosition)
    newSpread = std::max(mGoalPosition, spread + -kDofVelocity*_timeSincePreviousCall);
  else
    newSpread = std::min(mGoalPosition, spread + kDofVelocity*_timeSincePreviousCall);
  
  mSpreadDof->setPosition(newSpread);

}

}
}
