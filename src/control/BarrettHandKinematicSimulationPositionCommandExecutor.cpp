#include <exception>
#include <stdexcept>
#include <thread>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <aikido/control/BarrettHandKinematicSimulationPositionCommandExecutor.hpp>

namespace aikido {
namespace control {

constexpr std::chrono::milliseconds
    BarrettHandKinematicSimulationPositionCommandExecutor::kWaitPeriod;

//=============================================================================
BarrettHandKinematicSimulationPositionCommandExecutor::
    BarrettHandKinematicSimulationPositionCommandExecutor(
        dart::dynamics::SkeletonPtr robot,
        const std::string& prefix,
        ::dart::collision::CollisionDetectorPtr collisionDetector,
        ::dart::collision::CollisionGroupPtr collideWith)
  : mInExecution(false)
  , mCollisionDetector(std::move(collisionDetector))
  , mCollideWith(std::move(collideWith))
{
  if (mCollisionDetector && mCollideWith)
  {
    // If a collision group is given and its collision detector does not match
    // mCollisionDetector, set the collision group to an empty collision group.
    if (mCollisionDetector != mCollideWith->getCollisionDetector())
    {
      std::cerr << "[BarrettHandKinematicSimulationPositionCommandExecutor] "
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

  setupExecutors(std::move(robot), prefix);
}

//=============================================================================
void BarrettHandKinematicSimulationPositionCommandExecutor::setupExecutors(
    dart::dynamics::SkeletonPtr robot, const std::string& prefix)
{
  using dart::dynamics::Chain;
  using dart::dynamics::ChainPtr;
  using FingerPositionCommandExecutor = aikido::control::
      BarrettFingerKinematicSimulationPositionCommandExecutor;
  using FingerSpreadCommandExecutor
      = aikido::control::BarrettFingerKinematicSimulationSpreadCommandExecutor;

  if (prefix != "/left/" && prefix != "/right/")
  {
    std::stringstream message;
    message << "Invalid prefix '" << prefix << "', "
            << "must be either '/left/' or '/right/'";
    throw std::runtime_error(message.str());
  }

  const auto fingerChains = std::array<ChainPtr, 3> {{
    Chain::create(robot->getBodyNode(prefix + "finger0_0"), // finger0Spread
                  robot->getBodyNode(prefix + "finger0_2"), // finger0Distal
                  Chain::IncludeBoth),
    Chain::create(robot->getBodyNode(prefix + "finger1_0"), // finger1Spread
                  robot->getBodyNode(prefix + "finger1_2"), // finger1Distal
                  Chain::IncludeBoth),
    Chain::create(robot->getBodyNode(prefix + "finger2_1"), // finger2Primal
                  robot->getBodyNode(prefix + "finger2_2"), // finger2Distal
                  Chain::IncludeBoth),
  }};

  const auto spreadFingers = std::array<ChainPtr, 2>{{
    fingerChains[0],
    fingerChains[1]
  }};

  size_t spreadDof = 0;
  mSpreadCommandExecutor = std::make_shared<FingerSpreadCommandExecutor>(
      spreadFingers, spreadDof, mCollisionDetector, mCollideWith);

  constexpr auto primalDof = std::array<size_t, 3> {{1, 1, 0}};
  constexpr auto distalDof = std::array<size_t, 3> {{2, 2, 1}};
  for (size_t i = 0; i < fingerChains.size(); ++i)
  {
    mPositionCommandExecutors[i]
        = std::make_shared<FingerPositionCommandExecutor>(
            fingerChains[i],
            primalDof[i],
            distalDof[i],
            mCollisionDetector,
            mCollideWith);
  }
}

//=============================================================================
std::future<void>
BarrettHandKinematicSimulationPositionCommandExecutor::execute(
    const Eigen::VectorXd& goalPositions)
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
  mProximalGoalPositions = goalPositions.head<3>();
  mSpreadGoalPosition = goalPositions.row(3);
  mInExecution = true;
  mFingerFutures.clear();

  mFingerFutures.reserve(kNumPositionExecutor + kNumSpreadExecutor);
  for (size_t i = 0; i < kNumPositionExecutor; ++i)
    mFingerFutures.emplace_back(
        mPositionCommandExecutors[i]->execute(mProximalGoalPositions.row(i)));

  mFingerFutures.emplace_back(
      mSpreadCommandExecutor->execute(mSpreadGoalPosition));

  return mPromise->get_future();
}

//=============================================================================
void BarrettHandKinematicSimulationPositionCommandExecutor::step()
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInExecution)
    return;

  // Check whether fingers have completed.
  std::vector<std::future_status> statuses;
  std::exception_ptr expr;
  bool allFingersCompleted = true;

  for (size_t i = 0; i < mFingerFutures.size(); ++i)
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
    for (size_t i = 0; i < mFingerFutures.size(); ++i)
    {
      try
      {
        mFingerFutures[i].get();
      }
      catch (...)
      {
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
  for (int i = 0; i < kNumPositionExecutor; ++i)
    mPositionCommandExecutors[i]->step();

  mSpreadCommandExecutor->step();
}

//=============================================================================
bool BarrettHandKinematicSimulationPositionCommandExecutor::setCollideWith(
    ::dart::collision::CollisionGroupPtr collideWith)
{
  std::lock_guard<std::mutex> lockSpin(mMutex);

  if (mInExecution)
    return false;

  mCollideWith = std::move(collideWith);
  mCollisionDetector = mCollideWith->getCollisionDetector();
  return true;
}

} // namespace control
} // namespace aikido
