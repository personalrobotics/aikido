#ifndef AIKIDO_CONSTRAINT_INVERSEKINEMATICSSAMPLEABLEABLE_HPP_
#define AIKIDO_CONSTRAINT_INVERSEKINEMATICSSAMPLEABLEABLE_HPP_

#include <dart/dynamics/dynamics.hpp>
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include "Sampleable.hpp"

namespace aikido {
namespace constraint {

/// Transforms a SE3-Sampleable into a MetaSkeleton-Sampleable
/// that samples a configuration of a metaskeleton.
/// This samples a pose from a sampleable constraint
/// and uses an inverse kinematics solver to "lift" that pose
/// into the metaskeleton's configuration space. This class will retry a
/// configurable number of times if sampling from the provided sampleable
/// pose constraint or finding an inverse kinematic solution fails.
class InverseKinematicsSampleable : public Sampleable
{
public:
  /// Constructor.
  /// \param _stateSpace StateSpace of a skeleton
  ///        one of whose frame is being constrained by _poseConstraint.
  /// \param _poseConstraint This samples poses for a frame in the skeleton.
  /// \param _seedConstraint This samples configurations for the skeleton.
  ///        These samples are used as seeds when solving inverse kinematics.
  /// \param _inverseKinematics InverseKinematics solver for the pose being
  ///        sampled by _poseConstaint.
  /// \param _maxNumTrials Max number of trials for its sample generator
  ///        to retry sampling and finding an inverse kinematics solution.
  InverseKinematicsSampleable(
      statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      SampleablePtr _poseConstraint,
      SampleablePtr _seedConstraint,
      dart::dynamics::InverseKinematicsPtr _inverseKinematics,
      int _maxNumTrials);

  virtual ~InverseKinematicsSampleable() = default;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  SampleablePtr mPoseConstraint;
  SampleablePtr mSeedConstraint;
  dart::dynamics::InverseKinematicsPtr mInverseKinematics;
  int mMaxNumTrials;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_INVERSEKINEMATICSSAMPLEABLE_HPP_
