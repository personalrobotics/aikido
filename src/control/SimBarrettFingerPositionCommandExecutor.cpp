#include <aikido/control/SimBarrettFingerPositionCommandExecutor.hpp>
#include <thread>

namespace aikido{
namespace control{

//=============================================================================
SimBarrettFingerPositionCommandExecutor::SimBarrettFingerPositionCommandExecutor(
  ::dart::dynamics::ChainPtr _finger, int _proximal, int _distal,
  ::dart::collision::CollisionDetectorPtr _collisionDetector,
  ::dart::collision::CollisionGroupPtr _collideWith,
  ::dart::collision::CollisionOption _collisionOptions)
: mFinger(std::move(_finger))
, mProximalDof(nullptr)
, mDistalDof(nullptr)
, mCollisionDetector(std::move(_collisionDetector))
, mCollideWith(std::move(_collideWith))
, mCollisionOptions(std::move(_collisionOptions))
, mInExecution(false)
{
  if (!mFinger)
    throw std::invalid_argument("Finger is null.");

  if (_proximal == _distal)
    throw std::invalid_argument("proximal and distal dofs should be different.");

  const auto numDofs = mFinger->getNumDofs();

  if (static_cast<size_t>(_proximal) < numDofs)
    mProximalDof = mFinger->getDof(_proximal);

  if (static_cast<size_t>(_distal) < numDofs)
    mDistalDof = mFinger->getDof(_distal);

  if (!mProximalDof)
    throw std::invalid_argument("Finger does not have proximal dof.");

  if (!mDistalDof)
    throw std::invalid_argument("Finger does not have distal dof.");

  if (!mCollisionDetector)
    throw std::invalid_argument("CollisionDetctor is null.");

  if (!mCollideWith)
    throw std::invalid_argument("CollideWith is null.");


  mProximalCollisionGroup = mCollisionDetector->createCollisionGroup(
      mProximalDof->getChildBodyNode());

  mDistalCollisionGroup = mCollisionDetector->createCollisionGroup(
      mDistalDof->getChildBodyNode());

  mProximalLimits = mProximalDof->getPositionLimits();
  mDistalLimits = mDistalDof->getPositionLimits();
}

//=============================================================================
double SimBarrettFingerPositionCommandExecutor::getMimicRatio()
{
  return kMimicRatio;
}

//=============================================================================
std::future<void> SimBarrettFingerPositionCommandExecutor::execute(
  double _goalPosition)
{
  if (!mFinger->isAssembled())
    throw std::runtime_error("Finger is disassembled.");

  {
    std::lock_guard<std::mutex> lock(mMutex);

    if (mInExecution)
      throw std::runtime_error("Another command in execution.");

    mPromise.reset(new std::promise<void>());
    mInExecution = true;
    mDistalOnly = false;

    // Set mProximalGoalPosition.
    if (_goalPosition < mProximalLimits.first)
      mProximalGoalPosition = mProximalLimits.first;
    else if (_goalPosition > mProximalLimits.second)
      mProximalGoalPosition = mProximalLimits.second;
    else
      mProximalGoalPosition = _goalPosition;

    return mPromise->get_future();
  }
}

//=============================================================================
void SimBarrettFingerPositionCommandExecutor::terminate()
{
  mPromise->set_value();
  mInExecution = false;
  mCollideWith.reset();
}

//=============================================================================
void SimBarrettFingerPositionCommandExecutor::step(double _timeSincePreviousCall)
{
  using std::chrono::milliseconds;

  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInExecution)
    return;

  double distalPosition = mDistalDof->getPosition();
  double proximalPosition = mProximalDof->getPosition();

  // Check distal collision
  ::dart::collision::CollisionResult collisionResult;
  bool distalCollision = mCollisionDetector->collide(
    mDistalCollisionGroup.get(), mCollideWith.get(),
    mCollisionOptions, &collisionResult);

  if (distalCollision)
  {
    terminate();
    return;
  }

  double newDistal;
  bool distalLimitReached = false;

  if (proximalPosition < mProximalGoalPosition)
  { 
    newDistal = distalPosition + _timeSincePreviousCall*kDistalVelocity;
    if (mDistalLimits.second <= newDistal)
    {
      newDistal = mDistalLimits.second;
      distalLimitReached = true;
    }
  }
  else
  {
    newDistal = distalPosition - _timeSincePreviousCall*kDistalVelocity;
    if (mDistalLimits.first >= newDistal)
    {
      newDistal = mDistalLimits.first;
      distalLimitReached = true;
    }
  }

  mDistalDof->setPosition(newDistal);

  if (distalLimitReached)
  {
    terminate();
    return;
  }

  if (mDistalOnly)
    return;

  // Check proximal collision
  bool proximalCollision = mCollisionDetector->collide(
    mProximalCollisionGroup.get(), mCollideWith.get(),
    mCollisionOptions, &collisionResult);

  if (proximalCollision){
    mDistalOnly = true;
    return;
  }

  double newProximal;
  bool proximalGoalReached = false;
  if (proximalPosition < mProximalGoalPosition)
  {
    newProximal = proximalPosition + _timeSincePreviousCall*kProximalVelocity;
    if (mProximalGoalPosition <= newProximal)
    {
      newProximal = mProximalGoalPosition;
      proximalGoalReached = true;
    }
  }
  else
  {
    newProximal = proximalPosition - _timeSincePreviousCall*kProximalVelocity;
    if (mProximalGoalPosition >= newProximal)
    {
      newProximal = mProximalGoalPosition;
      proximalGoalReached = true;
    }
  }

  mProximalDof->setPosition(newProximal);

  if (proximalGoalReached)
  {
    terminate();
    return;
  }
}

//=============================================================================
bool SimBarrettFingerPositionCommandExecutor::setCollideWith(
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
