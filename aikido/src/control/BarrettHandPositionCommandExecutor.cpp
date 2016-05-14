#include <aikido/control/BarrettHandPositionCommandExecutor.hpp>
#include <thread>

namespace aikido{
namespace control{

//=============================================================================
BarrettHandPositionCommandExecutor::BarrettHandPositionCommandExecutor(
  std::vector<::dart::dynamics::ChainPtr> _fingers,
  std::chrono::milliseconds _cyclePeriod,
  ::dart::collision::CollisionDetectorPtr _collisionDetector,
  ::dart::collision::Option _collisionOptions)
: mFingers(std::move(_fingers))
, mCyclePeriod(_cyclePeriod)
, mCollisionDetector(std::move(_collisionDetector))
, mCollisionOptions(std::move(_collisionOptions))
, mSpinMutex()
, mRunning(true)
, mPromise(nullptr)
, mInExecution(false)
{

  if (mFingers.size() != numFingers){
    std::stringstream msg; 
    msg << "Need " << numFingers << "fingers."; 
    throw std::invalid_argument(msg.str());
  }

  for(int i = 0 ; i < numFingers; ++i)
  {
    if (!mFingers[i])
    {
      std::stringstream msg;
      msg << "Finger " << i << " is null.";
      throw std::invalid_argument(msg.str());
    }
  }

  if (mCyclePeriod.count() <= 0)
    throw std::invalid_argument("CyclePeriod should be positive.");

  if (!mCollisionDetector)
    throw std::invalid_argument("CollisionDetctor is null.");

  for (int i = 0; i < numFingers; ++i)
  {
    if (mFingers[i]->getNumDofs()!= numDofs[i])
    {
      std::stringstream msg; 
      msg << "Finger " << i << " should have " << numDofs[i] << "dofs.";
      throw std::invalid_argument(msg.str());
    }
  }

  // Set FingerCommandExecutors and collision groups
  mFingerCommandExecutors.reserve(numFingers);
  mPrimalCollisionGroups.reserve(numFingers); 
  mDistalCollisionGroups.reserve(numFingers); 

  for(int i = 0; i < numFingers ; ++i){
    int primal = numDofs[i]-2; 
    int distal = numDofs[i]-1;
    mFingerCommandExecutors.emplace_back(mFingers[i], primal, distal);

    mPrimalCollisionGroups.emplace_back(
      mCollisionDetector->createCollisionGroup(
        mFingers[i]->getBodyNode(primal)));

    mDistalCollisionGroups.emplace_back(
      mCollisionDetector->createCollisionGroup(
        mFingers[i]->getBodyNode(distal)));
  }

  mThread = std::thread(&BarrettHandPositionCommandExecutor::spin, this);

}

//=============================================================================
BarrettHandPositionCommandExecutor::~BarrettHandPositionCommandExecutor()
{
  {
    std::lock_guard<std::mutex> lockSpin(mSpinMutex);
    mRunning = false;
  }
  mCv.notify_one();
  
  mThread.join();
}

//=============================================================================
std::future<void> BarrettHandPositionCommandExecutor::execute(
  Eigen::Matrix<double, 4, 1> _positions,
  std::chrono::milliseconds _duration,
  ::dart::collision::CollisionGroupPtr _collideWith)
{
  if (_duration.count() <= 0)
    throw std::invalid_argument("Duration must be positive.");

  if (!_collideWith)
    throw std::invalid_argument("CollideWith is null.");

  Eigen::Vector3d setPoints = _positions.topRows(3); 
  double spread = _positions(3);

  // Map to upper/lower joint limits if the spread is above/below.
  if (spread < mFingers[0]->getDof(0)->getPositionLowerLimit())
    spread =  mFingers[0]->getDof(0)->getPositionLowerLimit();

  else if (spread > mFingers[0]->getDof(0)->getPositionUpperLimit())
    spread =  mFingers[0]->getDof(0)->getPositionUpperLimit();

  {
    std::lock_guard<std::mutex> lockSpin(mSpinMutex);

    if (mInExecution)
      throw std::runtime_error("Another command in execution.");

    mPromise.reset(new std::promise<void>());
    mSetPoints = setPoints; 
    mSpread = spread;
    mDuration = _duration;
    mCollideWith = _collideWith;
    mInExecution = true;
  }
  mCv.notify_one();

  return mPromise->get_future();
}

//=============================================================================
void BarrettHandPositionCommandExecutor::spin()
{
  using std::chrono::milliseconds; 
  using ::dart::collision::Result;

  // Values to be used when executing a command.
  int numIntervals;
  int currentStep;
  Eigen::Vector3d startPrimalAngles;
  Eigen::Vector3d angleIncrements;
  std::vector<bool> distalOnly(false, 3);
  double currentSpread; 
  double spreadIncrement;
  bool commandInExecution = false;

  while(true)
  {
    // Terminate the thread if mRunning is false.
    std::unique_lock<std::mutex> lockSpin(mSpinMutex);
    mCv.wait(lockSpin, [&] {
      // Reset values at the beginning of executing a command.
      if (!commandInExecution && mInExecution)
      {
        numIntervals = std::ceil(mDuration.count()/ mCyclePeriod.count());
        distalOnly = {false, false, false};

        // Set intervals, step-angles, step-spread
        for(int i = 0; i < 3; ++i)
          startPrimalAngles(i) = mFingers[i]->getDof(0)->getPosition();
        
        angleIncrements = (mSetPoints - startPrimalAngles)/numIntervals;

        currentSpread = mFingers[0]->getDof(0)->getPosition();
        spreadIncrement = (mSpread - currentSpread)/numIntervals;

        currentStep = 0; 

        commandInExecution = true;
      }

      return mInExecution || !mRunning; 
    }); 

    if (!mRunning)
    {
      if (mInExecution){
        mPromise->set_exception(std::make_exception_ptr(
          std::runtime_error("Command execution thread terminated"
          " while in execution.")));
      }
      break;
    }
    
    ++currentStep;
    Result collisionResult;

    // Move fingers 
    for(int i = 0; i < 3; ++i){
      try{
        mFingerCommandExecutors[i].execute(angleIncrements[i], distalOnly[i]);
      }catch (const std::runtime_error &e)
      {
        mPromise->set_exception(std::make_exception_ptr(
          std::runtime_error("Command execution thread terminated"
          " while in execution.")));
        break;
      }

      // Check primal collision 
      bool primalCollision = mCollisionDetector->collide(
        mPrimalCollisionGroups[i].get(), mCollideWith.get(),
        mCollisionOptions, collisionResult);

      if (primalCollision)
        distalOnly[i] = true;

      // Check distal collision
      bool distalCollision = mCollisionDetector->collide(
        mDistalCollisionGroups[i].get(), mCollideWith.get(),
        mCollisionOptions, collisionResult);

      if (distalCollision)
        angleIncrements(i) = 0;
    }

    // Move spread.
    mFingers[0]->getDof(0)->setPosition(currentSpread + spreadIncrement);
    mFingers[1]->getDof(0)->setPosition(currentSpread + spreadIncrement);
    currentSpread += spreadIncrement;

    // Termination condition
    if (currentStep >= numIntervals)
    {
      mPromise->set_value();
      mInExecution = false;
      commandInExecution = false;
    }

    std::this_thread::sleep_for(mCyclePeriod);
  }

}

}
}
