#include <aikido/constraint/IkSampleGenerator.hpp>
#include <dart/common/Console.h>
#include <math.h>
#include <vector>

using aikido::util::RNG;
using namespace dart::dynamics;

namespace aikido {
namespace constraint {

//=============================================================================
IkSampleGenerator::IkSampleGenerator(
  std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> _isometrySampler,
  const dart::dynamics::InverseKinematicsPtr& _ikPtr,
  std::unique_ptr<util::RNG> _rng,
  int _maxNumTrials)
: mIsometrySampler(std::move(_isometrySampler))
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

  if (!mIsometrySampler)
  {
    throw std::invalid_argument(
      "IsometrySampler is empty.");
  }

  if (_maxNumTrials <= 0)
  {
    throw std::invalid_argument(
      "MaxNumTrials is not positive.");
  }

};

//=============================================================================
boost::optional<Eigen::VectorXd> IkSampleGenerator::sample()
{

  // Sample TSR using rng and set it as the target for IKSolver
  bool success = false;

  // Check if Isometry3d can be sampled.
  if (!mIsometrySampler->canSample())
  {
    return boost::optional<Eigen::VectorXd>{};
  }

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
  return NO_LIMIT;
}

} // namespace constraint
} // namespace aikido
