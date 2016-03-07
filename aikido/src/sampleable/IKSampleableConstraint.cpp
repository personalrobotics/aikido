#include <aikido/sampleable/IKSampleable.hpp>

using namespace dart::dynamics;

namespace aikido {
namespace sampleable{


//=============================================================================
IKSampleableConstraint::IKSampleableConstraint(
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

};

//=============================================================================
IKSampleableConstraint::IKSampleableConstraint(
  const IKSampleableConstraint& other)
: mIsometry3dConstraintPtr(other.mIsometry3dConstraintPtr)
, mIKPtr(other.mIKPtr)
, mRng(std::move(other.mRng->clone()))
, mMaxNumTrials(other.mMaxNumTrials)
{
}

//=============================================================================
IKSampleableConstraint::IKSampleableConstraint(
  IKSampleableConstraint&& other)
: mIsometry3dConstraintPtr(other.mIsometry3dConstraintPtr)
, mIKPtr(other.mIKPtr)
, mRng(std::move(other.mRng->clone()))
, mMaxNumTrials(other.mMaxNumTrials)
{
}

//=============================================================================
IKSampleableConstraint& IKSampleableConstraint::operator=(
    const IKSampleableConstraint& other)
{
  mIsometry3dConstraintPtr = other.mIsometry3dConstraintPtr;
  mIKPtr = other.mIKPtr;
  mRng = std::move(other.mRng->clone());
  mMaxNumTrials = other.mMaxNumTrials;
  return *this;
}

//=============================================================================
IKSampleableConstraint& IKSampleableConstraint::operator=(
  IKSampleableConstraint&& other)
{
  mIsometry3dConstraintPtr = other.mIsometry3dConstraintPtr;
  mIKPtr = other.mIKPtr;
  mRng = std::move(other.mRng->clone());
  mMaxNumTrials = other.mMaxNumTrials;
  return *this;
}

//=============================================================================
std::unique_ptr<SampleGenerator<Eigen::VectorXd>> 
IKSampleableConstraint::createSampleGenerator() const
{
  return IKSampleGeneratorUniquePtr(
    new IKSampleGenerator(mIsometry3dConstraintPtr->createSampleGenerator(),
                          mIKPtr,
                          mRng->clone(),
                          mMaxNumTrials));
}

//=============================================================================
void IKSampleableConstraint::setRNG(std::unique_ptr<util::RNG> rng)
{
  mRng = std::move(rng);
}

} // namespace sampleable
} // namespace aikido

