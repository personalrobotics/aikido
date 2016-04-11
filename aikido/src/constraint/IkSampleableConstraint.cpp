#include <aikido/constraint/IkSampleableConstraint.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>
#include <aikido/constraint/IkSampleGenerator.hpp>

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

  if (!mInverseKinematics)
    throw std::invalid_argument("InverseKinematics is nullptr.");

  if (!mPoseConstraint)
    throw std::invalid_argument("Pose SampleableConstraint is nullptr.");

  if (!dynamic_cast<const statespace::SE3StateSpace*>(
        mPoseConstraint->getStateSpace().get()))
    throw std::invalid_argument(
      "Pose SampleableConstraint does not operate on an SE3StateSpace.");

  if (!mSeedConstraint)
    throw std::invalid_argument("Seed SampleableConstraint is nullptr.");

  if (mSeedConstraint->getStateSpace() != _stateSpace)
    throw std::invalid_argument(
      "Seed SampleableConstraint does not operate on this StateSpace.");

  if (mMaxNumTrials <= 0)
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
  return std::unique_ptr<IkSampleGenerator>(new IkSampleGenerator(
    mStateSpace,
    mInverseKinematics,
    mPoseConstraint->createSampleGenerator(),
    mSeedConstraint->createSampleGenerator(),
    mRng->clone(),
    mMaxNumTrials
  ));
}

//=============================================================================
void IkSampleableConstraint::setRNG(std::unique_ptr<util::RNG> rng)
{
  mRng = std::move(rng);
}

} // namespace constraint
} // namespace aikido

