#include "aikido/robot/GradientDescentIk.hpp"

#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"

using aikido::constraint::SampleGenerator;
using aikido::constraint::dart::createSampleableBounds;
using aikido::statespace::dart::MetaSkeletonStateSaver;

namespace aikido {
namespace robot {

//==============================================================================
GradientDescentIk::GradientDescentIk(
  ::dart::dynamics::MetaSkeletonPtr arm,
    ::dart::dynamics::BodyNodePtr endEffector,
    statespace::dart::ConstMetaSkeletonStateSpacePtr armSpace,
    common::RNG* rng) :
    mArm(arm), mEndEffector(endEffector), mArmSpace(armSpace),
    mSeedConfigSampleable(createSampleableBounds(mArmSpace, std::move(cloneRNGFrom(*rng)[0]))),
    mDartIkSolver(::dart::dynamics::InverseKinematics::create(mEndEffector))
{
  // Do nothing.
}

//==============================================================================
std::vector<Eigen::VectorXd> GradientDescentIk::getSolutions(
    const Eigen::Isometry3d& targetPose,
    const size_t maxSolutions
) const {
   auto saver = MetaSkeletonStateSaver(
      mArm, MetaSkeletonStateSaver::Options::POSITIONS);
  DART_UNUSED(saver);

  std::vector<Eigen::VectorXd> solutions;

  std::shared_ptr<SampleGenerator> ikSeedGenerator
      = mSeedConfigSampleable->createSampleGenerator();
  auto seedState = mArmSpace->createState();

  for (size_t i = 0; i < maxSolutions; i++)
  {
    // TODO: What to do if we can't sample a seed config?
    if (!ikSeedGenerator->sample(seedState))
      continue;

    mArmSpace->setState(mArm.get(), seedState);
    mDartIkSolver->getTarget()->setTransform(targetPose);
    bool withinTol = mDartIkSolver->solveAndApply(true);

    if (withinTol)
      solutions.push_back(mArm->getPositions());
  }

  return solutions;
}

} // namespace robot
} // namespace aikido