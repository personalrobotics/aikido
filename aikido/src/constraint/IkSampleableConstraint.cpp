#include <aikido/constraint/IkSampleableConstraint.hpp>
#include <aikido/statespace/SE3.hpp>

namespace aikido {
namespace constraint {

using statespace::SE3;
using statespace::dart::MetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSpace;
using statespace::SE3;
using dart::dynamics::INVALID_INDEX;


// For internal use only.
class IkSampleGenerator : public SampleGenerator
{
public:
  IkSampleGenerator(const IkSampleGenerator&) = delete;
  IkSampleGenerator(IkSampleGenerator&& other) = delete;

  IkSampleGenerator& operator=(const IkSampleGenerator& other) = delete;
  IkSampleGenerator& operator=(IkSampleGenerator&& other) = delete;

  virtual ~IkSampleGenerator() = default; 

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  bool sample(statespace::StateSpace::State* _state) override;

  // Documentation inherited.
  bool canSample() const override;

  // Documentation inherited.
  int getNumSamples() const override;

private:  
  // For internal use only.
  IkSampleGenerator(
    statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    dart::dynamics::InverseKinematicsPtr _inverseKinematics,
    std::unique_ptr<SampleGenerator> _poseSampler,
    std::unique_ptr<SampleGenerator> _seedSampler,
    int _maxNumTrials);

  statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  std::shared_ptr<statespace::SE3> mPoseStateSpace;
  dart::dynamics::InverseKinematicsPtr mInverseKinematics;
  std::unique_ptr<SampleGenerator> mPoseSampler;
  std::unique_ptr<SampleGenerator> mSeedSampler;
  int mMaxNumTrials;

  friend class IkSampleableConstraint;
};


//=============================================================================
IkSampleableConstraint::IkSampleableConstraint(
      MetaSkeletonStateSpacePtr _stateSpace,
      SampleableConstraintPtr _poseConstraint,
      SampleableConstraintPtr _seedConstraint,
      dart::dynamics::InverseKinematicsPtr _inverseKinematics,
      int _maxNumTrials)
  : mStateSpace(std::move(_stateSpace))
  , mPoseConstraint(std::move(_poseConstraint))
  , mSeedConstraint(std::move(_seedConstraint))
  , mInverseKinematics(std::move(_inverseKinematics))
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

  if (!dynamic_cast<SE3*>(mPoseConstraint->getStateSpace().get()))
    throw std::invalid_argument(
      "Pose SampleableConstraint does not operate on a SE3.");

  if (!mSeedConstraint)
    throw std::invalid_argument("Seed SampleableConstraint is nullptr.");

  if (mSeedConstraint->getStateSpace() != mStateSpace)
    throw std::invalid_argument(
      "Seed SampleGenerator is not for this StateSpace.");

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
    mMaxNumTrials
  ));
}

//=============================================================================
IkSampleGenerator::IkSampleGenerator(
      statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      dart::dynamics::InverseKinematicsPtr _inverseKinematics,
      std::unique_ptr<SampleGenerator> _poseSampler,
      std::unique_ptr<SampleGenerator> _seedSampler,
      int _maxNumTrials)
  : mStateSpace(std::move(_stateSpace))
  , mPoseStateSpace(
      std::dynamic_pointer_cast<SE3>(_poseSampler->getStateSpace()))
  , mInverseKinematics(std::move(_inverseKinematics))
  , mPoseSampler(std::move(_poseSampler))
  , mSeedSampler(std::move(_seedSampler))
  , mMaxNumTrials(_maxNumTrials)
{
  assert(mStateSpace);
  assert(mPoseStateSpace);
  assert(mInverseKinematics);
  assert(mPoseSampler);
  assert(mSeedSampler);
  assert(mSeedSampler->getStateSpace() == mStateSpace);
  assert(mMaxNumTrials > 0);

  if (mPoseSampler->getNumSamples() != NO_LIMIT)
    dtwarn << "[IkSampleGenerator::constructor] IkSampleGenerator only tries"
              " to find one IK solution per Isometry3d sample. The provided"
              " pose SampleGenerator contains a finite set of "
           << mPoseSampler->getNumSamples()
           << " samples. We advise against using this class on a finite"
              " sets of poses because they may be quickly exhausted.\n";

  if (mSeedSampler->getNumSamples() != NO_LIMIT)
    dtwarn << "[IkSampleGenerator::constructor] IkSampleGenerator uses up to "
           <<  mMaxNumTrials << " seeds per pose sample. The provided seed"
              " SampleGenerator contains a finite set of "
           << mSeedSampler->getNumSamples()
           << " samples. We advise against using this class on a finite"
              " sets of seeds because they may be quickly exhausted.\n";
}

//=============================================================================
statespace::StateSpacePtr IkSampleGenerator::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
bool IkSampleGenerator::sample(statespace::StateSpace::State* _state)
{
  if (!mSeedSampler->canSample() || !mPoseSampler->canSample())
    return false;

  auto seedState = mStateSpace->createState();
  auto poseState = mPoseStateSpace->createState();
  auto outputState = static_cast<MetaSkeletonStateSpace::State*>(_state);

  for(int i = 0; i < mMaxNumTrials; ++i)
  {
    // Sample a seed for the IK solver.
    // TODO: What should the retry logic look like if sampling a seed fails?
    if (!mSeedSampler->sample(seedState))
      continue;

    mStateSpace->setState(seedState);

    // Sample a goal for the IK solver.
    if (!mPoseSampler->sample(poseState))
      continue;

    mInverseKinematics->getTarget()->setTransform(poseState.getIsometry());

    // Run the IK solver. If it succeeds, return the solution.
    if (mInverseKinematics->solve(true))
    {
      mStateSpace->getState(outputState);
      return true;
    }
  }

  return false;
}

//=============================================================================
bool IkSampleGenerator::canSample() const
{
  return mSeedSampler->canSample() && mPoseSampler->canSample();
}

//=============================================================================
int IkSampleGenerator::getNumSamples() const
{
  return std::min(
    mSeedSampler->getNumSamples(), mPoseSampler->getNumSamples());
}

} // namespace constraint
} // namespace aikido

