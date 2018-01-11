#include <thread>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <aikido/control/BarrettFingerKinematicSimulationPositionCommandExecutor.hpp>

namespace aikido {
namespace control {

//==============================================================================
BarrettFingerKinematicSimulationPositionCommandExecutor::
    BarrettFingerKinematicSimulationPositionCommandExecutor(
        ::dart::dynamics::ChainPtr finger,
        std::size_t proximal,
        std::size_t distal,
        ::dart::collision::CollisionDetectorPtr collisionDetector,
        ::dart::collision::CollisionGroupPtr collideWith,
        ::dart::collision::CollisionOption collisionOptions)
  : mFinger(std::move(finger))
  , mProximalDof(nullptr)
  , mDistalDof(nullptr)
  , mCollisionDetector(std::move(collisionDetector))
  , mCollideWith(std::move(collideWith))
  , mCollisionOptions(std::move(collisionOptions))
  , mInExecution(false)
{
  if (!mFinger)
    throw std::invalid_argument("Finger is null.");

  if (proximal == distal)
    throw std::invalid_argument(
        "proximal and distal dofs should be different.");

  const auto numDofs = mFinger->getNumDofs();

  if (proximal < numDofs)
    mProximalDof = mFinger->getDof(proximal);

  if (distal < numDofs)
    mDistalDof = mFinger->getDof(distal);

  if (!mProximalDof)
    throw std::invalid_argument("Finger does not have proximal dof.");

  if (!mDistalDof)
    throw std::invalid_argument("Finger does not have distal dof.");

  if (mCollisionDetector && mCollideWith)
  {
    // If a collision group is given and its collision detector does not match
    // mCollisionDetector, set the collision group to an empty collision group.
    if (mCollisionDetector != mCollideWith->getCollisionDetector())
    {
      std::cerr << "[BarrettFingerKinematicSimulationPositionCommandExecutor] "
                << "CollisionDetector of type " << mCollisionDetector->getType()
                << " does not match CollisionGroup's CollisionDetector of type "
                << mCollideWith->getCollisionDetector()->getType() << std::endl;

      ::dart::collision::CollisionGroupPtr newCollideWith
          = mCollisionDetector->createCollisionGroup();
      for (auto i = 0u; i < mCollideWith->getNumShapeFrames(); ++i)
        newCollideWith->addShapeFrame(mCollideWith->getShapeFrame(i));
      mCollideWith = std::move(newCollideWith);
    }
  }
  else if (mCollisionDetector && !mCollideWith)
  {
    mCollideWith = mCollisionDetector->createCollisionGroup();
  }
  else if (!mCollisionDetector && mCollideWith)
  {
    mCollisionDetector = mCollideWith->getCollisionDetector();
  }
  else
  {
    // Default mCollisionDetector to FCL collision detector and mCollideWith to
    // empty collision group.
    mCollisionDetector = dart::collision::FCLCollisionDetector::create();
    mCollideWith = mCollisionDetector->createCollisionGroup();
  }

  setFingerCollisionGroup();

  mProximalLimits = mProximalDof->getPositionLimits();
  mDistalLimits = mDistalDof->getPositionLimits();
}

//==============================================================================
std::future<void>
BarrettFingerKinematicSimulationPositionCommandExecutor::execute(
    const Eigen::VectorXd& goalPosition)
{
  if (!mFinger->isAssembled())
    throw std::runtime_error("Finger is disassembled.");

  {
    std::lock_guard<std::mutex> lock(mMutex);
    double goalPositionValue = goalPosition[0];

    if (mInExecution)
      throw std::runtime_error("Another command in execution.");

    mPromise.reset(new std::promise<void>());
    mInExecution = true;
    mDistalOnly = false;
    mTimeOfPreviousCall = std::chrono::system_clock::now();

    // Set mProximalGoalPosition.
    if (goalPositionValue < mProximalLimits.first)
      mProximalGoalPosition = mProximalLimits.first;
    else if (goalPositionValue > mProximalLimits.second)
      mProximalGoalPosition = mProximalLimits.second;
    else
      mProximalGoalPosition = goalPositionValue;

    return mPromise->get_future();
  }
}

//==============================================================================
void BarrettFingerKinematicSimulationPositionCommandExecutor::terminate()
{
  mPromise->set_value();
  mInExecution = false;
}

//==============================================================================
void BarrettFingerKinematicSimulationPositionCommandExecutor::step()
{
  using namespace std::chrono;

  std::lock_guard<std::mutex> lock(mMutex);

  auto timeSincePreviousCall = system_clock::now() - mTimeOfPreviousCall;
  mTimeOfPreviousCall = system_clock::now();
  auto period = duration<double>(timeSincePreviousCall).count();

  if (!mInExecution)
    return;

  double distalPosition = mDistalDof->getPosition();
  double proximalPosition = mProximalDof->getPosition();

  // Check distal collision
  bool distalCollision = mCollisionDetector->collide(
      mDistalCollisionGroup.get(),
      mCollideWith.get(),
      mCollisionOptions,
      nullptr);

  if (distalCollision)
  {
    terminate();
    return;
  }

  double newDistal;
  bool distalLimitReached = false;

  if (proximalPosition < mProximalGoalPosition)
  {
    newDistal = distalPosition + period * kDistalSpeed;
    if (mDistalLimits.second <= newDistal)
    {
      newDistal = mDistalLimits.second;
      distalLimitReached = true;
    }
  }
  else
  {
    newDistal = distalPosition - period * kDistalSpeed;
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
      mProximalCollisionGroup.get(),
      mCollideWith.get(),
      mCollisionOptions,
      nullptr);

  if (proximalCollision)
  {
    mDistalOnly = true;
    return;
  }

  double newProximal;
  bool proximalGoalReached = false;
  if (proximalPosition < mProximalGoalPosition)
  {
    newProximal = proximalPosition + period * kProximalSpeed;
    if (mProximalGoalPosition <= newProximal)
    {
      newProximal = mProximalGoalPosition;
      proximalGoalReached = true;
    }
  }
  else
  {
    newProximal = proximalPosition - period * kProximalSpeed;
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

//==============================================================================
bool BarrettFingerKinematicSimulationPositionCommandExecutor::setCollideWith(
    ::dart::collision::CollisionGroupPtr collideWith)
{
  std::lock_guard<std::mutex> lockSpin(mMutex);

  if (mInExecution)
    return false;

  mCollideWith = std::move(collideWith);
  mCollisionDetector = mCollideWith->getCollisionDetector();

  setFingerCollisionGroup();

  return true;
}

//==============================================================================
void BarrettFingerKinematicSimulationPositionCommandExecutor::
    setFingerCollisionGroup()
{
  if (mProximalCollisionGroup
      && mProximalCollisionGroup->getCollisionDetector() == mCollisionDetector
      && mDistalCollisionGroup
      && mDistalCollisionGroup->getCollisionDetector() == mCollisionDetector)
    return;

  mProximalCollisionGroup = mCollisionDetector->createCollisionGroup(
      mProximalDof->getChildBodyNode());
  mDistalCollisionGroup = mCollisionDetector->createCollisionGroup(
      mDistalDof->getChildBodyNode());
}

} // namespace control
} // namespace aikido
