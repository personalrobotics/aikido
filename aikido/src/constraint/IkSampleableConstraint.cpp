#include <aikido/constraint/IkSampleableConstraint.hpp>
//#include <aikido/constraint/IkSampleGenerator.hpp>

using namespace dart::dynamics;

namespace aikido {
namespace constraint {

//=============================================================================
IkSampleableConstraint::IkSampleableConstraint(
      statespace::MetaSkeletonStateSpacePtr _stateSpace,
      SampleableConstraintPtr _delegateConstraint,
      dart::dynamics::InverseKinematicsPtr _inverseKinematics,
      std::unique_ptr<util::RNG> _rng,
      int _maxNumTrials)
  : mStateSpace(std::move(_stateSpace))
  , mConstraint(std::move(_delegateConstraint))
  , mInverseKinematics(std::move(_inverseKinematics))
  , mRng(std::move(_rng))
  , mMaxNumTrials(_maxNumTrials)
{
  if (!mRng)
  {
    throw std::invalid_argument(
      "Random generator is empty.");
  }

  if (!_inverseKinematics)
  {
    throw std::invalid_argument(
      "IKPtr is empty.");
  }

  if (!_delegateConstraint)
  {
    throw std::invalid_argument(
      "IsometryConstraint is empty.");
  }

  if (_maxNumTrials <= 0)
  {
    throw std::invalid_argument(
      "MaxNumTrials is not positive.");
  }
}

//=============================================================================
IkSampleableConstraint::IkSampleableConstraint(
      const IkSampleableConstraint& other)
  : IkSampleableConstraint(
      other.mStateSpace,
      other.mConstraint,
      other.mInverseKinematics,
      other.mRng->clone(),
      other.mMaxNumTrials
    )
{
}

//=============================================================================
IkSampleableConstraint::IkSampleableConstraint(
      IkSampleableConstraint&& other)
  : IkSampleableConstraint(
      std::move(other.mStateSpace),
      std::move(other.mConstraint),
      std::move(other.mInverseKinematics),
      std::move(other.mRng),
      other.mMaxNumTrials
    )
{
}

//=============================================================================
IkSampleableConstraint& IkSampleableConstraint::operator=(
    const IkSampleableConstraint& other)
{
  mStateSpace = other.mStateSpace;
  mConstraint = other.mConstraint;
  mInverseKinematics = other.mInverseKinematics;
  mRng = mRng->clone();
  return *this;
}

//=============================================================================
IkSampleableConstraint& IkSampleableConstraint::operator=(
  IkSampleableConstraint&& other)
{
  mStateSpace = std::move(other.mStateSpace);
  mConstraint = std::move(other.mConstraint);
  mInverseKinematics = std::move(other.mInverseKinematics);
  mRng = std::move(mRng);
  return *this;
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

