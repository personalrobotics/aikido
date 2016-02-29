#include <aikido/sampleable/IKSampleable.hpp>
#include <math.h>
#include <vector>

using aikido::util::RNG;
using dart::dynamics::SimpleFrame;
using dart::dynamics::Frame;
using dart::dynamics::DegreeOfFreedom;
using SkeletonPtr = std::shared_ptr<::dart::dynamics::Skeleton>;

namespace aikido {
namespace sampleable{

//=============================================================================
boost::optional<Eigen::VectorXd> IKSampleGenerator::sample()
{

  // Sample TSR using rng and set it as the target for IKSolver
  bool success = false;

  for(int i = 0; i < mMaxNumTrials; i++)
  {
    /// Create an Isometry3d sample.
    boost::optional<Eigen::Isometry3d> isometryOptional = mIsometrySampler->sample();
    if (!isometryOptional)
    {
      continue;
    }

    // Set end-effector to match sample.
    Eigen::Isometry3d isometrySample = isometryOptional.get();
    mIKPtr->getTarget()->setTransform(isometrySample);

    // Get active joints' lower and upper limits.
    const std::vector<size_t> activeDofIndices = mIKPtr->getDofs();
    SkeletonPtr skeletonPtr = mIKPtr->getNode()->getSkeleton();
    std::vector<double> jointLowerLimits;
    std::vector<double> jointUpperLimits;
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

        jointLowerLimits.push_back(lower);
        jointUpperLimits.push_back(upper);
      }
    }

    /// Distributions to sample joint dof values.
    std::vector<std::uniform_real_distribution<double>> distributions;
    for(int i = 0; i < activeDofIndices.size(); i++)
    {
      std::uniform_real_distribution<double> distribution(jointLowerLimits.at(i),
                                                          jointUpperLimits.at(i));
      distributions.push_back(distribution);
    }


    // Generate random seed for each joint and solve IK. 
    std::shared_ptr<::dart::optimizer::Problem> problem = mIKPtr->getProblem();
    for(int j = 0; j < mMaxNumTrials; j++)
    {
      Eigen::VectorXd seed(activeDofIndices.size());

      for(int k = 0; k < activeDofIndices.size(); k++)
      {
        seed(k) = distributions.at(k)(*mRng);
      }
      
      // Initialize the joints with seed.
      mIKPtr->setPositions(seed);
      Eigen::VectorXd solution;
      bool success = mIKPtr->solve(solution, false);
      if (success)
      {
        return solution;
      }
    }
  }
  
  return boost::optional<Eigen::VectorXd>{};
}


//=============================================================================
bool IKSampleGenerator::canSample() const
{
  return true;
}

//=============================================================================
int IKSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

} // sampleable
} // aikido
