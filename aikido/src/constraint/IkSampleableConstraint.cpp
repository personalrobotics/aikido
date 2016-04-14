#include <aikido/constraint/IkSampleableConstraint.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>
#include <aikido/constraint/IkSampleGenerator.hpp>

using namespace dart::dynamics;

namespace aikido {
namespace constraint {

using statespace::SE3StateSpace;
using statespace::MetaSkeletonStateSpacePtr;

//=============================================================================
IkSampleableConstraint::IkSampleableConstraint(
      MetaSkeletonStateSpacePtr _stateSpace,
      SampleableConstraintPtr _poseConstraint,
      SampleableConstraintPtr _seedConstraint,
      dart::dynamics::InverseKinematicsPtr _inverseKinematics,
      std::unique_ptr<util::RNG> _rng,
      int _maxNumTrials)
  : mStateSpace(std::move(_stateSpace))
  , mPoseConstraint(_poseConstraint)  // std::move crashes.
  , mSeedConstraint(std::move(_seedConstraint))
  , mInverseKinematics(std::move(_inverseKinematics))
  , mRng(std::move(_rng))
  , mMaxNumTrials(_maxNumTrials)
{
  if (!mStateSpace)
    throw std::invalid_argument("MetaSkeletonStateSpace is nullptr.");

  if (!mInverseKinematics)
    throw std::invalid_argument("InverseKinematics is nullptr.");

  const auto stateMetaSkeleton = mStateSpace->getMetaSkeleton();
  const auto ikSkeleton = mInverseKinematics->getNode()->getSkeleton();
  for (const size_t dofIndex : mInverseKinematics->getDofs())
  {
    const auto dof = ikSkeleton->getDof(dofIndex);
    if (stateMetaSkeleton->getIndexOf(dof, false) == INVALID_INDEX)
    {
      std::stringstream msg;
      msg << "DegreeOfFreedom '" << dof->getName() << "' is used by the"
             " InverseKinematics solver, but is absent from the"
             " MetaSkeletonStateSpace this constraint is defined over.";
      throw std::invalid_argument(msg.str());
    }
  }

  if (!mPoseConstraint)
    throw std::invalid_argument("Pose SampleGenerator is nullptr.");

  if (!dynamic_cast<SE3StateSpace*>(_poseConstraint->getStateSpace().get()))
    throw std::invalid_argument(
      "Pose SampleableConstraint does not operate on a SE3StateSpace.");

  if (!mSeedConstraint)
    throw std::invalid_argument("Seed SampleableConstraint is nullptr.");

  if (mSeedConstraint->getStateSpace() != mStateSpace)
    throw std::invalid_argument(
      "Seed SampleGenerator is not for this StateSpace.");

  if (!mRng)
    throw std::invalid_argument("RNG is nullptr.");

  if (mMaxNumTrials <= 0)
    throw std::invalid_argument("Maximum number of trials must be positive.");
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

