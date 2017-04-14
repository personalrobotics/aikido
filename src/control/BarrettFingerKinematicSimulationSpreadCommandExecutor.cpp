#include <aikido/control/BarrettFingerKinematicSimulationSpreadCommandExecutor.hpp>
#include <thread>
#include <exception>

namespace aikido{
namespace control{

//=============================================================================
BarrettFingerKinematicSimulationSpreadCommandExecutor::BarrettFingerKinematicSimulationSpreadCommandExecutor(
  std::array<::dart::dynamics::ChainPtr, 2> fingers, size_t spread,
  ::dart::collision::CollisionGroupPtr collideWith,
  ::dart::collision::CollisionOption collisionOptions)
: mFingers(std::move(fingers))
, mCollideWith(std::move(collideWith))
, mCollisionOptions(std::move(collisionOptions))
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
    if (static_cast<size_t>(spread) >= numDofs)
      throw std::invalid_argument("Finger does not have spread dof.");

    mSpreadDofs.push_back(mFingers[i]->getDof(spread));
    if (!mSpreadDofs[i]){
      std::stringstream msg;
      msg << i << "th finger does not have spread dof.";
      throw std::invalid_argument(msg.str());
    }
  }

  if (!mCollideWith)
    throw std::invalid_argument("CollideWith is null.");
  mCollisionDetector = mCollideWith->getCollisionDetector();

  mSpreadCollisionGroup = mCollisionDetector->createCollisionGroup();
  for (auto finger: mFingers)
  {
    for (auto body: finger->getBodyNodes())
    mSpreadCollisionGroup->addShapeFramesOf(body);
  }

  mDofLimits = mSpreadDofs[0]->getPositionLimits();

  for (size_t i = 1; i < kNumFingers; ++i)
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
std::future<void> BarrettFingerKinematicSimulationSpreadCommandExecutor::execute(
  double goalPosition)
{
  for(size_t i = 0; i < kNumFingers; ++i)
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

    if (goalPosition < mDofLimits.first)
      mGoalPosition = mDofLimits.first;
    else if (goalPosition > mDofLimits.second)
      mGoalPosition = mDofLimits.second;
    else
      mGoalPosition = goalPosition;

    mInExecution = true;

    return mPromise->get_future();
  }
}

//=============================================================================
void BarrettFingerKinematicSimulationSpreadCommandExecutor::step(
  const std::chrono::milliseconds& timeSincePreviousCall)
{
  auto period = std::chrono::duration<double>(
    timeSincePreviousCall).count();

  // Terminate the thread if mRunning is false.
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInExecution)
    return;

  // Current spread. Check that all spreads have same values.
  double spread = mSpreadDofs[0]->getPosition();
  for(size_t i = 1; i < kNumFingers; ++i)
  {
    double _spread = mSpreadDofs[i]->getPosition();
    if (std::abs(spread - _spread) > kDofTolerance)
    {
      std::stringstream msg;
      msg << i << "th finger dof value not equal to 0th finger dof. "
      << "Expecting " << spread << ", got " << spread;
      auto expr = std::make_exception_ptr(std::runtime_error(msg.str()));
      mPromise->set_exception(expr);
      mInExecution = false;
      return;
    }
  }

  // Check collision
  bool collision = mCollisionDetector->collide(
    mSpreadCollisionGroup.get(), mCollideWith.get(),
    mCollisionOptions, nullptr);

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
    newSpread = std::max(mGoalPosition,
      spread + -kDofSpeed*period);
  else
    newSpread = std::min(mGoalPosition,
      spread + kDofSpeed*period);

  for (auto spreadDof: mSpreadDofs)
    spreadDof->setPosition(newSpread);
}

//=============================================================================
bool BarrettFingerKinematicSimulationSpreadCommandExecutor::setCollideWith(
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
