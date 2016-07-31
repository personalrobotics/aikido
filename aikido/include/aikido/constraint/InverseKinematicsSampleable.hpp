#ifndef AIKIDO_CONSTRAINT_INVERSEKINEMATICSSAMPLEABLEABLE_HPP_
#define AIKIDO_CONSTRAINT_INVERSEKINEMATICSSAMPLEABLEABLE_HPP_

#include "Sampleable.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include <dart/dynamics/dynamics.hpp>
#include <Eigen/Dense>

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

  /// Constructor for solving IK for single SE3 pose.
  /// Uses current configuration as seed for solving IK. 
  /// \param _stateSpace Statespace of a skeleton
  ///         containing the target frame to solve for
  /// \param _pose Target SE3 pose of target frame
  /// \param _inverseKinematics InverseKinematics solver for _pose
  InverseKinematicsSampleable(
    statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    const Eigen::Isometry3d& _pose,
    dart::dynamics::InverseKinematicsPtr _inverseKinematics);


  virtual ~InverseKinematicsSampleable() = default;

  // Documentation inherited.
  virtual statespace::StateSpacePtr getStateSpace() const override;

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
