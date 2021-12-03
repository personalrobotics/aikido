#include "aikido/robot/GradientDescentIk.hpp"

#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"

using aikido::constraint::SampleGenerator;
using aikido::statespace::dart::MetaSkeletonStateSaver;

namespace aikido {
namespace robot {

//==============================================================================
GradientDescentIk::GradientDescentIk()
{
  // TODO.
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
      = mSeedSampleable->createSampleGenerator();
  auto seedState = mArmSpace->createState();

  // How many times the IK solver will re-sample a single solution if it
  // is out of tolerance.
  for (int i = 0; i < maxSolutions; i++)
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