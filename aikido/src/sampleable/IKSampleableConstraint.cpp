#include <aikido/sampleable/IKSampleable.hpp>

using namespace dart::dynamics;

namespace aikido {
namespace sampleable{


//=============================================================================
IKSampleableConstraint::IKSampleableConstraint(
  const std::shared_ptr<SampleableConstraint<Eigen::Isometry3d>> 
    _isometry3dConstraint,
  const dart::dynamics::InverseKinematicsPtr _ikPtr,
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

