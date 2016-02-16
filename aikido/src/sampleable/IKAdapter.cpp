#include <aikido/sampleable/IKAdapter.hpp>

#include <stdexcept>
#include <math.h>
#include <vector>

using aikido::util::RNG;
using dart::dynamics::SimpleFrame;
using dart::dynamics::Frame;

namespace aikido {
namespace sampleable{

//=============================================================================
IKAdapter::IKAdapter(const TSRPtr& _tsrPtr,
                     const InverseKinematicsPtr& _ikPtr)
  : mTSRPtr(_tsrPtr), mIKPtr(_ikPtr)
{

}

//=============================================================================
const Eigen::VectorXd IKAdapter::sample(RNG& rng)
{
  // Sample TSR using rng and set it as the target for IKSolver
  Eigen::Isometry3d tsrSample = mTSRPtr->sample(rng);
  auto simpleFrame = std::make_shared<SimpleFrame>(Frame::World());
  simpleFrame->setTransform(tsrSample);
  mIKPtr->setTarget(simpleFrame);

  // Sample random DOF values based on rng
  std::vector<size_t> activeDofIndices = mIKPtr->getDofs();
  Eigen::VectorXd initialGuess(Eigen::VectorXd::Zero(activeDofIndices.size()));
  SkeletonPtr skeletonPtr = mIKPtr->getNode()->getSkeleton();
  for(int i = 0; i < activeDofIndices.size(); i++)
  {
    double lower = skeletonPtr->getPositionLowerLimit(activeDofIndices.at(i));
    double upper = skeletonPtr->getPositionUpperLimit(activeDofIndices.at(i));
    initialGuess(i) = _sample(rng, lower, upper);
  }  

  // Initialize the problem with the randomly generated sample
  std::shared_ptr<::dart::optimizer::Problem> problem = mIKPtr->getProblem();
  problem->clearAllSeeds();
  problem->setInitialGuess(initialGuess);

  Eigen::VectorXd solution;
  mIKPtr->solve(solution, false);
  return solution;
}

//============================================================================
double IKAdapter::_sample(RNG& rng, double lower, double upper) const
{
  
  std::uniform_real_distribution<double>  distribution(lower, upper);
  double sample = distribution(rng);

  return sample;
}
//=============================================================================
bool IKAdapter::isSatisfied(const Eigen::VectorXd state) const
{

  mIKPtr->setPositions(state);

  // Get end-effector transform w.r.t. world origin
  Eigen::Isometry3d tsr = mIKPtr->getNode()->getWorldTransform();

  return mTSRPtr->isSatisfied(tsr);
}

//=============================================================================
bool IKAdapter::canSample() const
{
  return true;
}
//=============================================================================
int IKAdapter::maxSampleCount() const
{
  return SampleableRegion::INFTY;
}


} // namespace sampleable
} // namespace aikido
