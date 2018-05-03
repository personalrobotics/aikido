#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/statespace/SE3.hpp"

#undef dtwarn
#define dtwarn (::dart::common::colorErr("Warning", __FILE__, __LINE__, 33))

namespace aikido {
namespace constraint {
namespace dart {

using statespace::dart::MetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSpace;
using statespace::SE3;
using ::dart::dynamics::INVALID_INDEX;

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
      statespace::dart::MetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaskeleton,
      ::dart::dynamics::InverseKinematicsPtr inverseKinematics,
      std::unique_ptr<SampleGenerator> poseSampler,
      std::unique_ptr<SampleGenerator> seedSampler,
      int maxNumTrials,
      bool seedCurrentConfiguration);

  statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  std::shared_ptr<statespace::SE3> mPoseStateSpace;
  ::dart::dynamics::InverseKinematicsPtr mInverseKinematics;
  std::unique_ptr<SampleGenerator> mPoseSampler;
  std::unique_ptr<SampleGenerator> mSeedSampler;
  int mMaxNumTrials;
  bool mSeedCurrentConfiguration;

  friend class InverseKinematicsSampleable;
};

//==============================================================================
InverseKinematicsSampleable::InverseKinematicsSampleable(
    MetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaskeleton,
    SampleablePtr poseConstraint,
    SampleablePtr seedConstraint,
    ::dart::dynamics::InverseKinematicsPtr inverseKinematics,
    int maxNumTrials,
    bool seedCurrentConfiguration)
  : mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(metaskeleton))
  , mPoseConstraint(std::move(poseConstraint))
  , mSeedConstraint(std::move(seedConstraint))
  , mInverseKinematics(std::move(inverseKinematics))
  , mMaxNumTrials(maxNumTrials)
  , mSeedCurrentConfiguration(seedCurrentConfiguration)
{
  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("MetaSkeletonStateSpace is nullptr.");

  if (!mMetaSkeleton)
    throw std::invalid_argument("_metaskeleton is nullptr.");

  if (!mInverseKinematics)
    throw std::invalid_argument("InverseKinematics is nullptr.");

  const auto ikSkeleton = mInverseKinematics->getNode()->getSkeleton();

  if (mInverseKinematics->getDofs().empty())
  {
    throw std::invalid_argument(
        "Zero degrees of freedom for InverseKinematics solver.");
  }

  for (const std::size_t dofIndex : mInverseKinematics->getDofs())
  {
    const auto dof = ikSkeleton->getDof(dofIndex);
    if (mMetaSkeleton->getIndexOf(dof, false) == INVALID_INDEX)
    {
      std::stringstream msg;
      msg << "DegreeOfFreedom '" << dof->getName()
          << "' is used by the"
             " InverseKinematics solver, but is absent from the"
             " MetaSkeleton this constraint is defined with.";
      throw std::invalid_argument(msg.str());
    }
  }

  if (!mPoseConstraint)
    throw std::invalid_argument("Pose SampleGenerator is nullptr.");

  if (!dynamic_cast<SE3*>(mPoseConstraint->getStateSpace().get()))
    throw std::invalid_argument("Pose Sampleable does not operate on a SE3.");

  if (!mSeedConstraint)
    throw std::invalid_argument("Seed Sampleable is nullptr.");

  if (mSeedConstraint->getStateSpace() != mMetaSkeletonStateSpace)
    throw std::invalid_argument(
        "Seed SampleGenerator is not for this StateSpace.");

  if (mMaxNumTrials <= 0)
    throw std::invalid_argument("Maximum number of trials must be positive.");
}

//==============================================================================
statespace::StateSpacePtr InverseKinematicsSampleable::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
std::unique_ptr<SampleGenerator>
InverseKinematicsSampleable::createSampleGenerator() const
{
  return std::unique_ptr<IkSampleGenerator>(
      new IkSampleGenerator(
          mMetaSkeletonStateSpace,
          mMetaSkeleton,
          mInverseKinematics,
          mPoseConstraint->createSampleGenerator(),
          mSeedConstraint->createSampleGenerator(),
          mMaxNumTrials,
          mSeedCurrentConfiguration));
}

//==============================================================================
IkSampleGenerator::IkSampleGenerator(
    statespace::dart::MetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaskeleton,
    ::dart::dynamics::InverseKinematicsPtr inverseKinematics,
    std::unique_ptr<SampleGenerator> poseSampler,
    std::unique_ptr<SampleGenerator> seedSampler,
    int maxNumTrials,
    bool seedCurrentConfiguration)
  : mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(metaskeleton))
  , mPoseStateSpace(
        std::dynamic_pointer_cast<SE3>(poseSampler->getStateSpace()))
  , mInverseKinematics(std::move(inverseKinematics))
  , mPoseSampler(std::move(poseSampler))
  , mSeedSampler(std::move(seedSampler))
  , mMaxNumTrials(maxNumTrials)
  , mSeedCurrentConfiguration(seedCurrentConfiguration)
{
  assert(mMetaSkeletonStateSpace);
  assert(mMetaSkeleton);
  assert(mPoseStateSpace);
  assert(mInverseKinematics);
  assert(mPoseSampler);
  assert(mSeedSampler);
  assert(mSeedSampler->getStateSpace() == mMetaSkeletonStateSpace);
  assert(mMaxNumTrials > 0);

  if (mPoseSampler->getNumSamples() != NO_LIMIT)
  {
    dtwarn << "[IkSampleGenerator::constructor] IkSampleGenerator only tries"
           << " to find one IK solution per Isometry3d sample. The provided"
           << " pose SampleGenerator contains a finite set of "
           << mPoseSampler->getNumSamples()
           << " samples. We advise against using this class on a finite"
           << " sets of poses because they may be quickly exhausted.\n";
  }

  if (mSeedSampler->getNumSamples() != NO_LIMIT)
  {
    dtwarn << "[IkSampleGenerator::constructor] IkSampleGenerator uses up to "
           << mMaxNumTrials << " seeds per pose sample. The provided seed"
           << " SampleGenerator contains a finite set of "
           << mSeedSampler->getNumSamples()
           << " samples. We advise against using this class on a finite"
           << " sets of seeds because they may be quickly exhausted.\n";
  }
}

//==============================================================================
statespace::StateSpacePtr IkSampleGenerator::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
bool IkSampleGenerator::sample(statespace::StateSpace::State* _state)
{
  auto poseState = mPoseStateSpace->createState();
  auto outputState = static_cast<MetaSkeletonStateSpace::State*>(_state);

  if (!mPoseSampler->canSample())
    return false;

  if (mSeedCurrentConfiguration)
  {
    // Sample a goal for the IK solver.
    if (mPoseSampler->sample(poseState))
    {
      mInverseKinematics->getTarget()->setTransform(poseState.getIsometry());

      // Run the IK solver. If it succeeds, return the solution.
      if (mInverseKinematics->solve(true))
      {
        mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), outputState);
        return true;
      }
    }
  }

  auto seedState = mMetaSkeletonStateSpace->createState();
  if (!mSeedSampler->canSample())
    return false;

  for (int i = 1; i < mMaxNumTrials; ++i)
  {
    // Sample a seed for the IK solver.
    // TODO: What should the retry logic look like if sampling a seed fails?
    if (!mSeedSampler->sample(seedState))
      continue;

    mMetaSkeletonStateSpace->setState(mMetaSkeleton.get(), seedState);

    if (!mPoseSampler->sample(poseState))
      continue;

    mInverseKinematics->getTarget()->setTransform(poseState.getIsometry());

    if (mInverseKinematics->solve(true))
    {
      mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), outputState);
      return true;
    }
  }
  return false;
}

//==============================================================================
bool IkSampleGenerator::canSample() const
{
  return mSeedSampler->canSample() && mPoseSampler->canSample();
}

//==============================================================================
int IkSampleGenerator::getNumSamples() const
{
  return std::min(mSeedSampler->getNumSamples(), mPoseSampler->getNumSamples());
}

} // namespace dart
} // namespace constraint
} // namespace aikido
