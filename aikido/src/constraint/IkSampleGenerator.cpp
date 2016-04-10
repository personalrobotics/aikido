#include <aikido/constraint/IkSampleGenerator.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>
#include <boost/format.hpp>
#include <dart/common/Console.h>
#include <math.h>
#include <vector>

using boost::format;
using boost::str;
using aikido::util::RNG;
using namespace dart::dynamics;

namespace aikido {
namespace constraint {

//=============================================================================
IkSampleGenerator::IkSampleGenerator(
      statespace::MetaSkeletonStateSpacePtr _stateSpace,
      dart::dynamics::InverseKinematicsPtr _inverseKinematics,
      std::unique_ptr<SampleGenerator> _delegateSampler,
      std::unique_ptr<util::RNG> _rng,
      int _maxNumTrials)
  : mStateSpace(std::move(_stateSpace))
  , mInverseKinematics(std::move(_inverseKinematics))
  , mDelegateSampler(std::move(_delegateSampler))
  , mRng(std::move(_rng))
  , mMaxNumTrials(_maxNumTrials)
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpace is nullptr.");

  if (!mInverseKinematics)
    throw std::invalid_argument("InverseKinematics is nullptr.");

  if (!mRng)
    throw std::invalid_argument("RNG is nullptr.");

  if (!mIsometrySampler)
    throw std::invalid_argument("SampleGenerator nullptr.");

  if (mMaxNumTrials <= 0)
    throw std::invalid_argument(str(
      format("maximum number of trials must be positive; got %d.")
      % mMaxNumTrials));

  if (mIsometrySampler->getNumSamples() != NO_LIMIT)
    dtwarn << "[IkSampleGenerator::constructor] IkSampleGenerator only tries"
              " to find one IK solution per Isometry3d sample. The provided"
              " SampleGenerator contains a finite set of "
           << mIsometrySampler->getNumSamples()
           << " samples. We advise against using this class on a finite"
              " sets of poses because they may be quickly exhausted.\n";
};

//=============================================================================
statespace::StateSpacePtr IkSampleGenerator::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
bool IkSampleGenerator::sample(statespace::StateSpace::State* _state)
{
  // Sample TSR using rng and set it as the target for IKSolver
  bool success = false;

  // Check if Isometry3d can be sampled.
  if (!mIsometrySampler->canSample())
    return false;

  // Get active joints' lower and upper limits.
  const std::vector<size_t> activeDofIndices = mIKPtr->getDofs();
  std::vector<double> jointLowerLimits;
  std::vector<double> jointUpperLimits;
  jointUpperLimits.reserve(activeDofIndices.size());
  jointLowerLimits.reserve(activeDofIndices.size());
  SkeletonPtr skeletonPtr = mIKPtr->getNode()->getSkeleton();
  
  for(int j = 0; j < activeDofIndices.size(); j++)
  {
    DegreeOfFreedom* dof = skeletonPtr->getDof(activeDofIndices.at(j));
    double lower, upper;
    if (dof->isCyclic())
    {
      lower = -M_PI;
      upper = M_PI;
    }
    else
    {
      lower = dof->getPositionLowerLimit();
      upper = dof->getPositionUpperLimit();

      // Check for unbounded joint.
      if (!std::isfinite(lower) || !std::isfinite(upper))
      {
        dterr << "Unbounded joint.\n";
        return boost::optional<Eigen::VectorXd>{};
      }
    }

    jointLowerLimits.emplace_back(lower);
    jointUpperLimits.emplace_back(upper);
  }

  // Sample random configuration and call IKSolver. 
  for(int i = 0; i < mMaxNumTrials; i++)
  {
    // Create an Isometry3d sample.
    boost::optional<Eigen::Isometry3d> isometry = mIsometrySampler->sample();
    if (!isometry)
    {
      continue;
    }
    // Set the Isometry sample as IKPtr's target.
    mIKPtr->getTarget()->setTransform(isometry.get());

    // Distributions to sample joint dof values.
    std::vector<std::uniform_real_distribution<double>> distributions;
    distributions.reserve(activeDofIndices.size());
    for(int i = 0; i < activeDofIndices.size(); i++)
    {
      std::uniform_real_distribution<double> distribution(
        jointLowerLimits.at(i),
        jointUpperLimits.at(i));
      distributions.emplace_back(distribution);
    }

    // Generate random seed for each joint
    Eigen::VectorXd seed(activeDofIndices.size());
    for(int j = 0; j < activeDofIndices.size(); j++)
    {
      seed(j) = distributions.at(j)(*mRng);
    }
    mIKPtr->setPositions(seed);

    // Solve IK.
    Eigen::VectorXd solution;
    bool success = mIKPtr->solve(solution, false);
    if (success)
    {
      return solution;
    }
  }
  
  return boost::optional<Eigen::VectorXd>{};
}

//=============================================================================
bool IkSampleGenerator::canSample() const
{
  return mIsometrySampler->canSample();
}

//=============================================================================
int IkSampleGenerator::getNumSamples() const
{
  return mIsometrySampler->getNumSamples();
}

} // namespace constraint
} // namespace aikido
