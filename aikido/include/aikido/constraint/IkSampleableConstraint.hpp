#ifndef AIKIDO_CONSTRAINT_IKSAMPLEABLECONSTRAINT_H
#define AIKIDO_CONSTRAINT_IKSAMPLEABLECONSTRAINT_H

#include "Sampleable.hpp"
#include "../statespace/MetaSkeletonStateSpace.hpp"
#include <dart/dynamics/dynamics.h>

namespace aikido {
namespace constraint {

/// Transforms a Isometry3d SampleableConstraint into a VectorXd
/// SampleableConstraint that represents the positions of a Skeleton by
/// sampling an Isometry3d and using inverse kinematics to "lift" that pose
/// into the Skeleton's configuration space. This class will retry a
/// configurable number of times if sampling from the Isometry3d constaint or
/// finding an inverse kinematic solution fails.
class IkSampleableConstraint : public SampleableConstraint
{
public:
  IkSampleableConstraint(
    statespace::MetaSkeletonStateSpacePtr _stateSpace,
    SampleableConstraintPtr _poseConstraint,
    SampleableConstraintPtr _seedConstraint,
    dart::dynamics::InverseKinematicsPtr _inverseKinematics,
    std::unique_ptr<util::RNG> _rng,
    int _maxNumTrials);

  virtual ~IkSampleableConstraint() = default;

  // Documentation inherited.
  virtual statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;
  
  /// Set rng seed.
  void setRNG(std::unique_ptr<util::RNG> rng);
  
private:
  statespace::MetaSkeletonStateSpacePtr mStateSpace;
  SampleableConstraintPtr mPoseConstraint;
  SampleableConstraintPtr mSeedConstraint;
  dart::dynamics::InverseKinematicsPtr mInverseKinematics;
  std::unique_ptr<util::RNG> mRng;

  int mMaxNumTrials;
};

} // namespace constraint 
} // namespace aikido

#endif // AIKIDO_Constraint_IKSAMPLEABLECONSTRAINT_H
