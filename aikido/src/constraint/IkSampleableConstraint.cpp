#include <aikido/constraint/IkSampleableConstraint.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>
//#include <aikido/constraint/IkSampleGenerator.hpp>

using namespace dart::dynamics;

namespace aikido {
namespace constraint {

//=============================================================================
IkSampleableConstraint::IkSampleableConstraint(
      statespace::MetaSkeletonStateSpacePtr _stateSpace,
      SampleableConstraintPtr _poseConstraint,
      SampleableConstraintPtr _seedConstraint,
      dart::dynamics::InverseKinematicsPtr _inverseKinematics,
      std::unique_ptr<util::RNG> _rng,
      int _maxNumTrials)
  : mStateSpace(std::move(_stateSpace))
  , mPoseConstraint(std::move(_poseConstraint))
  , mSeedConstraint(std::move(_seedConstraint))
  , mInverseKinematics(std::move(_inverseKinematics))
  , mRng(std::move(_rng))
  , mMaxNumTrials(_maxNumTrials)
{
  if (!mRng)
    throw std::invalid_argument("RNG is nullptr.");

  if (!_inverseKinematics)
    throw std::invalid_argument("InverseKinematics is nullptr.");

  if (!_poseConstraint)
    throw std::invalid_argument("Pose SampleableConstraint is nullptr.");

  if (!dynamic_cast<const statespace::SE3StateSpace*>(
        _poseConstraint->getStateSpace().get()))
    throw std::invalid_argument(
      "Pose SampleableConstraint does not operate on an SE3StateSpace.");

  if (!_seedConstraint)
    throw std::invalid_argument("Seed SampleableConstraint is nullptr.");

  if (_seedConstraint->getStateSpace() != _stateSpace)
    throw std::invalid_argument(
      "Seed SampleableConstraint does not operate on this StateSpace.");

  if (_maxNumTrials <= 0)
    throw std::invalid_argument("Maximum number of trials is not positive.");
}

//=============================================================================
statespace::StateSpacePtr IkSampleableConstraint::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
std::unique_ptr<SampleGenerator>
  IkSampleableConstraint::createSampleGenerator() const
{
#if 0
  return std::unique_ptr<IkSampleGenerator>(new IkSampleGenerator(
    mIsometry3dConstraintPtr->createSampleGenerator(),
    mInverseKinematics, mRng->clone(), mMaxNumTrials));
#endif
  return nullptr;
}

//=============================================================================
void IkSampleableConstraint::setRNG(std::unique_ptr<util::RNG> rng)
{
  mRng = std::move(rng);
}

} // namespace constraint
} // namespace aikido

