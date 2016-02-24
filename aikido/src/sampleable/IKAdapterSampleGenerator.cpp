#include <aikido/sampleable/IKAdapterSampleGenerator.hpp>

#include <math.h>
#include <vector>

using aikido::util::RNG;
using dart::dynamics::SimpleFrame;
using dart::dynamics::Frame;
using dart::dynamics::DegreeOfFreedom;

namespace aikido {
namespace sampleable{

//=============================================================================
optional<Eigen::VectorXd> IKAdapterSampleGenerator::sample()
{

  // Sample TSR using rng and set it as the target for IKSolver
  int num_trials = 0; 
  bool success = false;
  while(num_trials < 5){
    optional<Eigen::Isometry3d> isometryOptional = mIsometry3dConstraintPtr->sampler()->sample();
    if (isometryOptional)
    {
      Eigen::Isometry3d isometrySample = isometryOptional.get();
      mIKPtr->getTarget()->setTransform(isometrySample);
      success = true;
      break;
    }
    num_trials++;
  }
  if (!success)
  {
    return optional<Eigen::VectorXd>{};
  }

  // Get active joints' lower and upper limits.
  const std::vector<size_t> activeDofIndices = mIKPtr->getDofs();
  SkeletonPtr skeletonPtr = mIKPtr->getNode()->getSkeleton();
  std::vector<double> jointLowerLimits;
  std::vector<double> jointUpperLimits;
  for(int i = 0; i < activeDofIndices.size(); i++)
  {
    DegreeOfFreedom* dof = skeletonPtr->getDof(activeDofIndices.at(i));
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
        return optional<Eigen::VectorXd>{};
      }

      jointLowerLimits.push_back(lower);
      jointUpperLimits.push_back(upper);
    }
  }

  // Generate random seeds and solve IK. 
  num_trials = 0;
  std::shared_ptr<::dart::optimizer::Problem> problem = mIKPtr->getProblem();
  while(num_trials < 5)
  {
    Eigen::VectorXd seed(Eigen::VectorXd::Zero(activeDofIndices.size()));

    for(int i = 0; i < activeDofIndices.size(); i++)
    {
      std::uniform_real_distribution<double> distribution(jointLowerLimits.at(i),
                                                          jointUpperLimits.at(i));
      seed(i) = distribution(*mRng.get());
    }
    
    // Initialize the problem with seed.
    problem->clearAllSeeds();
    problem->setInitialGuess(seed);

    Eigen::VectorXd solution;
    bool success = mIKPtr->solve(solution, false);
    if (success)
    {
      return solution;
    }

  }
  
  return optional<Eigen::VectorXd>{};
}


//=============================================================================
bool IKAdapterSampleGenerator::canSample()
{
  return true;
}

//=============================================================================
int IKAdapterSampleGenerator::numSamples()
{
  return NO_LIMIT;
}

} // sampleable
} // aikido
