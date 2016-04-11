#ifndef AIKIDO_CONSTRAINT_IKSAMPLEGENERATOR_H
#define AIKIDO_CONSTRAINT_IKSAMPLEGENERATOR_H

#include "IkSampleableConstraint.hpp"
#include "Sampleable.hpp"
#include "../statespace/SE3StateSpace.hpp"

namespace aikido {
namespace constraint {

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
    statespace::MetaSkeletonStateSpacePtr _stateSpace,
    dart::dynamics::InverseKinematicsPtr _inverseKinematics,
    std::unique_ptr<SampleGenerator> _poseSampler,
    std::unique_ptr<SampleGenerator> _seedSampler,
    std::unique_ptr<util::RNG> _rng,
    int _maxNumTrials);

  statespace::MetaSkeletonStateSpacePtr mStateSpace;
  std::shared_ptr<statespace::SE3StateSpace> mPoseStateSpace;
  dart::dynamics::InverseKinematicsPtr mInverseKinematics;
  std::unique_ptr<SampleGenerator> mPoseSampler;
  std::unique_ptr<SampleGenerator> mSeedSampler;
  std::unique_ptr<util::RNG> mRng;
  int mMaxNumTrials;

  friend class IkSampleableConstraint;
};

} // namespace constraint 
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_IKSAMPLEGENERATOR_H
