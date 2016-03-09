#include <aikido/constraint/IkSampleableConstraint.hpp>
#include <aikido/constraint/IkSampleGenerator.hpp>

using namespace dart::dynamics;

namespace aikido {
namespace constraint {


//=============================================================================
IkSampleableConstraint::IkSampleableConstraint(
  const SampleablePoseConstraint& _isometry3dConstraint,
  const dart::dynamics::InverseKinematicsPtr& _ikPtr,
  std::unique_ptr<util::RNG> _rng,
  int _maxNumTrials)
: mIsometry3dConstraintPtr(_isometry3dConstraint)
, mIKPtr(_ikPtr)
, mRng(std::move(_rng))
, mMaxNumTrials(_maxNumTrials)
{
  if (!mRng)
  {
    throw std::invalid_argument(
      "Random generator is empty.");
  }

  if (!_ikPtr)
  {
    throw std::invalid_argument(
      "IKPtr is empty.");
  }

  if (!_isometry3dConstraint)
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
: IkSampleableConstraint(other.mIsometry3dConstraintPtr,
                         other.mIKPtr,
                         other.mRng->clone(),
                         other.mMaxNumTrials)
{
}

//=============================================================================
IkSampleableConstraint::IkSampleableConstraint(
  IkSampleableConstraint&& other)
: IkSampleableConstraint(other.mIsometry3dConstraintPtr,
                         other.mIKPtr,
                         other.mRng->clone(),
                         other.mMaxNumTrials)
{
}

//=============================================================================
IkSampleableConstraint& IkSampleableConstraint::operator=(
    const IkSampleableConstraint& other)
{
  mIsometry3dConstraintPtr = other.mIsometry3dConstraintPtr;
  mIKPtr = other.mIKPtr;
  mRng = std::move(other.mRng->clone());
  mMaxNumTrials = other.mMaxNumTrials;
  return *this;
}

//=============================================================================
IkSampleableConstraint& IkSampleableConstraint::operator=(
  IkSampleableConstraint&& other)
{
  mIsometry3dConstraintPtr = other.mIsometry3dConstraintPtr;
  mIKPtr = other.mIKPtr;
  mRng = std::move(other.mRng->clone());
  mMaxNumTrials = other.mMaxNumTrials;
  return *this;
}

//=============================================================================
std::unique_ptr<SampleGenerator<Eigen::VectorXd>> 
IkSampleableConstraint::createSampleGenerator() const
{
  return std::unique_ptr<IkSampleGenerator>(new IkSampleGenerator(
    mIsometry3dConstraintPtr->createSampleGenerator(),
    mIKPtr, mRng->clone(), mMaxNumTrials));
}

//=============================================================================
void IkSampleableConstraint::setRNG(std::unique_ptr<util::RNG> rng)
{
  mRng = std::move(rng);
}

} // namespace constraint
} // namespace aikido

