#include <aikido/control/BarrettFingerSpreadCommandExecutor.hpp>
#include <thread>
#include <exception>

namespace aikido{
namespace control{

//=============================================================================
BarrettFingerSpreadCommandExecutor::BarrettFingerSpreadCommandExecutor(
  std::array<::dart::dynamics::ChainPtr, 2> _fingers, int _spread, 
  ::dart::collision::CollisionDetectorPtr _collisionDetector,
  ::dart::collision::CollisionOption _collisionOptions)
: mFingers(std::move(_fingers))
, mCollisionDetector(std::move(_collisionDetector))
, mCollisionOptions(std::move(_collisionOptions))
, mInExecution(false)
{
  if (mFingers.size() != kNumFingers)
  {
    std::stringstream msg; 
    msg << "Expecting " << kNumFingers << " fingers;"
    << "got << " << mFingers.size() << "."; 
    throw std::invalid_argument(msg.str());
  }
  
  mSpreadDofs.reserve(kNumFingers);
  for (int i=0; i < kNumFingers; ++i)
  {
    if(!mFingers[i])
    {
      std::stringstream msg; 
      msg << i << "th finger is null.";
      throw std::invalid_argument(msg.str());
    }

    const auto numDofs = mFingers[i]->getNumDofs();
    if (_spread >= numDofs)
      throw std::invalid_argument("Finger does not have spread dof.");

    mSpreadDofs.push_back(mFingers[i]->getDof(_spread));
    if (!mSpreadDofs[i]){
      std::stringstream msg;
      msg << i << "th finger does not have spread dof.";
      throw std::invalid_argument(msg.str());
    }
  }

  if (!mCollisionDetector)
    throw std::invalid_argument("CollisionDetector is null.");

  mSpreadCollisionGroup = mCollisionDetector->createCollisionGroup();
  for (auto finger: mFingers)
  {
    for (auto body: finger->getBodyNodes())
    mSpreadCollisionGroup->addShapeFramesOf(body);
  }

  mDofLimits = mSpreadDofs[0]->getPositionLimits();

  for (int i=1; i < kNumFingers; ++i)
  {
    auto limits = mSpreadDofs[i]->getPositionLimits();
    if (std::abs(limits.first - mDofLimits.first) > kDofTolerance)
    {
      std::stringstream msg; 
      msg << "LowerJointLimit for the fingers should be the same. " 
      << "Expecting " << mDofLimits.first << "; got " << limits.first;
      throw std::invalid_argument(msg.str());
    }  

    if (std::abs(limits.second - mDofLimits.second) > kDofTolerance)
    {
      std::stringstream msg; 
      msg << "UpperJointLimit for the fingers should be the same. " 
      << "Expecting " << mDofLimits.second << "; got " << limits.second;
      throw std::invalid_argument(msg.str());
    }  
  }
}

//=============================================================================
std::future<void> BarrettFingerSpreadCommandExecutor::execute(
  double _goalPosition,
  ::dart::collision::CollisionGroupPtr _collideWith)
{
  if (!_collideWith)
    throw std::invalid_argument("CollideWith is null.");

  for(int i=0; i < kNumFingers; ++i)
  {
    if (!mFingers[i]->isAssembled())
    {
      std::stringstream msg; 
      msg << i << "th finger is no longer linked.";
      throw std::runtime_error(msg.str());  
    }
  }
  
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

    mCollideWith = _collideWith;
    mInExecution = true;

    return mPromise->get_future();
  }
}

//=============================================================================
void BarrettFingerSpreadCommandExecutor::step(double _timeSincePreviousCall)
{
  using std::chrono::milliseconds; 

  // Terminate the thread if mRunning is false.
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInExecution)
    return;
  
  // Current spread. Check that all spreads have same values.
  double spread = mSpreadDofs[0]->getPosition();
  for(int i=1; i < kNumFingers; ++i)
  {
    double _spread = mSpreadDofs[i]->getPosition();
    if (std::abs(spread - _spread) > kDofTolerance)
    {
      std::stringstream msg; 
      msg << i << "th finger dof value not equal to 0th finger dof. " 
      << "Expecting " << spread << ", got " << _spread;
      auto expr = std::make_exception_ptr(std::runtime_error(msg.str()));
      mPromise->set_exception(expr);
      mInExecution = false;
      return;
    }
  }
  
  // Check collision 
  ::dart::collision::CollisionResult collisionResult;
  bool collision = mCollisionDetector->collide(
    mSpreadCollisionGroup.get(), mCollideWith.get(),
    mCollisionOptions, &collisionResult);

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
  
  for (auto spreadDof: mSpreadDofs)
    spreadDof->setPosition(newSpread);

}

}
}
