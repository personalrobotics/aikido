#include <aikido/control/BarrettFingerPositionCommandExecutor.hpp>
#include <thread>

namespace aikido{
namespace control{

//=============================================================================
BarrettFingerPositionCommandExecutor::BarrettFingerPositionCommandExecutor(
  ::dart::dynamics::ChainPtr _finger, int _primal, int _distal,
  ::dart::collision::CollisionDetectorPtr _collisionDetector,
  ::dart::collision::Option _collisionOptions)
: mFinger(std::move(_finger))
, mCollisionDetector(std::move(_collisionDetector))
, mCollisionOptions(std::move(_collisionOptions))
, mInExecution(false)
{
  if (!mFinger)
    throw std::invalid_argument("Finger is null.");

  if (_primal == _distal)
    throw std::invalid_argument("Primal and distal dofs should be different.");

  mPrimalDof = mFinger->getDof(_primal);
  mDistalDof = mFinger->getDof(_distal);

  if (!mPrimalDof)
    throw std::invalid_argument("Finger does not have primal dof.");

  if (!mDistalDof)
    throw std::invalid_argument("Finger does not have distal dof.");

  if (!mCollisionDetector)
    throw std::invalid_argument("CollisionDetctor is null.");

  mPrimalCollisionGroup = mCollisionDetector->createCollisionGroup(
      mPrimalDof->getChildBodyNode());

  mDistalCollisionGroup = mCollisionDetector->createCollisionGroup(
      mDistalDof->getChildBodyNode());

  mPrimalLimits = mPrimalDof->getPositionLimits();
  mDistalLimits = mDistalDof->getPositionLimits();
}

//=============================================================================
double BarrettFingerPositionCommandExecutor::getMimicRatio() 
{
  return kMimicRatio;
}

//=============================================================================
std::future<void> BarrettFingerPositionCommandExecutor::execute(
  double _goalPosition, ::dart::collision::CollisionGroupPtr _collideWith)
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
    mCollideWith = _collideWith;
    mInExecution = true;
    mDistalOnly = false;

    // Set mPrimalGoalPosition and mDistalGoalPosition.
    if (_goalPosition < mPrimalLimits.first)
      mPrimalGoalPosition = mPrimalLimits.first;
    else if (_goalPosition > mPrimalLimits.second)
      mPrimalGoalPosition = mPrimalLimits.second;
    else
      mPrimalGoalPosition = _goalPosition;

    double distalGoalPosition = mPrimalGoalPosition*kMimicRatio;
    if (distalGoalPosition < mDistalLimits.first)
      mDistalGoalPosition = mDistalLimits.first;
    else if (distalGoalPosition > mDistalLimits.second)
      mDistalGoalPosition = mDistalLimits.second;
    else
      mDistalGoalPosition = distalGoalPosition;

    return mPromise->get_future();
  }
}

//=============================================================================
void BarrettFingerPositionCommandExecutor::step(double _timeSincePreviousCall)
{
  using std::chrono::milliseconds; 
  using ::dart::collision::Result;

  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInExecution)
    return;
    
  double distalPosition = mDistalDof->getPosition();
  double primalPosition = mPrimalDof->getPosition();

  // Check distal collision
  Result collisionResult;
  bool distalCollision = mCollisionDetector->collide(
    mDistalCollisionGroup.get(), mCollideWith.get(),
    mCollisionOptions, collisionResult);

  if (distalCollision || 
      std::abs(mDistalGoalPosition - distalPosition) < kTolerance)
  {
    mPromise->set_value();
    mInExecution = false;
    mCollideWith.reset();
    return;
  }

  double newDistal;
  if (distalPosition < mDistalGoalPosition)
    newDistal = std::min(mDistalGoalPosition, 
                         distalPosition + _timeSincePreviousCall*kDistalVelocity);
  else
    newDistal = std::max(mDistalGoalPosition,
                         distalPosition - _timeSincePreviousCall*kDistalVelocity);

  mDistalDof->setPosition(newDistal);

  if (mDistalOnly)
    return;

  // Check primal collision 
  bool primalCollision = mCollisionDetector->collide(
    mPrimalCollisionGroup.get(), mCollideWith.get(),
    mCollisionOptions, collisionResult);

  if (primalCollision){
    mDistalOnly = true;
    return;
  }

  double newPrimal;
  if (primalPosition < mPrimalGoalPosition)
    newPrimal = std::min(mPrimalGoalPosition,
                         primalPosition + _timeSincePreviousCall*kPrimalVelocity);
  else
    newPrimal = std::max(mPrimalGoalPosition,
                         primalPosition - _timeSincePreviousCall*kPrimalVelocity);

  mPrimalDof->setPosition(newPrimal);

}

}
}
