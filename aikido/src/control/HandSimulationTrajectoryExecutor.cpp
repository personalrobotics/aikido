#include <aikido/control/HandSimulationTrajectoryExecutor.hpp>
#include <thread>

namespace aikido{
namespace control{

//=============================================================================
HandSimulationTrajectoryExecutor::HandSimulationTrajectoryExecutor(
  std::vector<::dart::dynamics::ChainPtr> _fingers,
  std::chrono::milliseconds _cyclePeriod,
  ::dart::collision::CollisionDetectorPtr _collisionDetector,
  ::dart::collision::CollisionGroupPtr _collideWith,
  ::dart::collision::Option _collisionOptions)
: mFingers(std::move(_fingers))
, mCyclePeriod(_cyclePeriod)
, mCollisionDetector(std::move(_collisionDetector))
, mCollideWith(std::move(_collideWith))
, mCollisionOptions(_collisionOptions)
{
  using ::dart::dynamics::Chain;

  if (mFingers.size() != 3)
    throw std::invalid_argument("Need three fingers.");

  for(int i = 0 ; i < 3; ++i)
  {
    if (!mFingers[i])
    {
      std::stringstream msg;
      msg << "Finter " << i << " is null.";
      throw std::invalid_argument(msg.str());
    }
  }

  if (mCyclePeriod.count() <= 0)
    throw std::invalid_argument("CyclePeriod should be positive.");

  if (!mCollisionDetector)
    throw std::invalid_argument("CollisionDetctor is null.");

  if (!mCollideWith)
    throw std::invalid_argument("CollideWith is null.");

  mFingerPrimalDofExecutors.reserve(3);
  mSubFingers.reserve(3);

  // Fingers
  for (int i = 0; i < 2; ++i)
  {    
    if (mFingers[i]->getNumDofs()!= 3)
      throw std::invalid_argument("First two fingers should have 3 dofs.");
    auto subFinger = Chain::create(
      mFingers[i]->getBodyNode(0), mFingers[i]->getBodyNode(2));
    mSubFingers.emplace_back(subFinger);
  }
  mSubFingers.emplace_back(mFingers[2]);

  // StepExecutors
  for(int i = 0; i < 3; ++i){
    mFingerPrimalDofExecutors.emplace_back(
      FingerSimulationStepExecutor(mSubFingers[i]));
  }

  if (mFingers[2]->getNumDofs() != 2)
    throw std::invalid_argument("Third finger should have 2 dofs.");

  // Finger primal and distal Collision groups 
  mPrimalCollisionGroups.reserve(3); 
  mDistalCollisionGroups.reserve(3); 

  for(int i = 0; i < 3; ++i)
  {
    mPrimalCollisionGroups.emplace_back(
      mCollisionDetector->createCollisionGroup(
        mSubFingers[i]->getBodyNode(0)));

    mDistalCollisionGroups.emplace_back(
      mCollisionDetector->createCollisionGroup(
        mSubFingers[i]->getBodyNode(1)));
  }

}

//=============================================================================
void HandSimulationTrajectoryExecutor::execute(
  std::vector<double> _set_points, 
  double _spread, std::chrono::milliseconds _duration)
{
  using std::chrono::milliseconds; 
  using ::dart::collision::Result;
    
  if (_set_points.size() != 3)
    throw std::invalid_argument("Provide 3 set points.");

  // Map to upper/lower joint limits if the setpoints are above/below.
  for (int i = 0; i < 3; ++i)
  {
    if (_set_points[i] < mSubFingers[i]->getDof(0)->getPositionLowerLimit())
      _set_points[i] = mSubFingers[i]->getDof(0)->getPositionLowerLimit();

    else if (_set_points[i] > mSubFingers[i]->getDof(0)->getPositionUpperLimit())
      _set_points[i] = mSubFingers[i]->getDof(0)->getPositionUpperLimit();
  }

  if (_spread < mFingers[0]->getDof(0)->getPositionLowerLimit())
    _spread =  mFingers[0]->getDof(0)->getPositionLowerLimit();

  else if (_spread > mFingers[0]->getDof(0)->getPositionUpperLimit())
    _spread =  mFingers[0]->getDof(0)->getPositionUpperLimit();
  

  if (_duration.count() <= 0)
    throw std::invalid_argument("Duration must be positive.");

  // Set intervals, step-angles, step-spread
  int numIntervals = std::ceil(_duration.count()/ mCyclePeriod.count());
  double currentPrimalAngles[3];
  double angleIncrements[3];
  bool distalOnly[3] = {false, false, false};

  for(int i = 0; i < 3; ++i)
  {
    currentPrimalAngles[i] = mSubFingers[i]->getDof(0)->getPosition();
    angleIncrements[i] = (_set_points[i] - currentPrimalAngles[i])/numIntervals;
  }

  double currentSpread = mFingers[0]->getDof(0)->getPosition();
  double spreadIncrement = (_spread - currentSpread)/numIntervals;

  Result collisionResult;

  // Increment joint angle.
  for(int i = 0 ; i < numIntervals; ++i)
  {
    // Move fingers 
    for(int i = 0; i < 3; ++i){
      mFingerPrimalDofExecutors[i].execute(angleIncrements[i], distalOnly[i]);
      currentPrimalAngles[i] += angleIncrements[i]; 
    
      // Check priaml collision 
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
        angleIncrements[i] = 0;
    }

    // Spread.
    mFingers[0]->getDof(0)->setPosition(currentSpread + spreadIncrement);
    mFingers[1]->getDof(0)->setPosition(currentSpread + spreadIncrement);
    currentSpread += spreadIncrement;

    std::this_thread::sleep_for(mCyclePeriod);
  }
}



}
}
