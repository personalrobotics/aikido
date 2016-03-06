#include <aikido/sampleable/IKSampleable.hpp>
#include <math.h>
#include <vector>

using aikido::util::RNG;
using namespace dart::dynamics;

namespace aikido {
namespace sampleable{

//=============================================================================
IKSampleGenerator::IKSampleGenerator(
  std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> _isometrySampler,
  const dart::dynamics::InverseKinematicsPtr _ikPtr,
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

};

//=============================================================================
boost::optional<Eigen::VectorXd> IKSampleGenerator::sample()
{

  // Sample TSR using rng and set it as the target for IKSolver
  bool success = false;

  // Check if Isometry3d can be sampled.
  if (!mIsometrySampler->canSample())
  {
    return boost::optional<Eigen::VectorXd>{};
  }

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

        // Check for unbounded acyclic joint.
        if (lower > upper)
        {
          std::cout << "Unbounded acyclic joint. Using [-PI,PI)." << std::endl;
          lower = -M_PI; 
          upper = M_PI;
        }
      }

      jointLowerLimits.emplace_back(lower);
      jointUpperLimits.emplace_back(upper);

    }

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

    // Generate random seed for each joint and solve IK. 
    std::shared_ptr<::dart::optimizer::Problem> problem = mIKPtr->getProblem();
    problem->clearAllSeeds();

    for(int j = 0; j < mMaxNumTrials; j++)
    {
      Eigen::VectorXd seed(activeDofIndices.size());
      for(int k = 0; k < activeDofIndices.size(); k++)
      {
        seed(k) = distributions.at(k)(*mRng);
      }
      problem->addSeed(seed);
    }

    // IKPtr will first try with current joint values and then use the seeds.    
    mIKPtr->getSolver()->setNumMaxIterations(mMaxNumTrials);

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
bool IKSampleGenerator::canSample() const
{
  return mIsometrySampler->canSample();
}

//=============================================================================
int IKSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

} // sampleable
} // aikido
