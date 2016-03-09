#ifndef AIKIDO_CONSTRAINT_IKSAMPLEABLECONSTRAINT_H
#define AIKIDO_CONSTRAINT_IKSAMPLEABLECONSTRAINT_H

#include "Sampleable.hpp"
#include <dart/dynamics/dynamics.h>

namespace aikido {
namespace constraint {

/// Transforms a Isometry3d SampleableConstraint into a VectorXd
/// SampleableConstraint that represents the positions of a Skeleton by
/// sampling an Isometry3d and using inverse kinematics to "lift" that pose
/// into the Skeleton's configuration space. This class will retry a
/// configurable number of times if sampling from the Isometry3d constaint or
/// finding an inverse kinematic solution fails.
class IkSampleableConstraint : public SampleableConstraint<Eigen::VectorXd>
{
public:
  using SampleablePoseConstraint =
    std::shared_ptr<SampleableConstraint<Eigen::Isometry3d>>;

  /// Constructor.
  ///
  /// \param[in] _isometry3dConstraint pose constraint to lift
  /// \param[in] _ikPtr inverse kinematics solver to use for lifting
  /// \param[in] _rng random number generator used by sample generators
  /// \param[in] _maxNumTrials number of retry attempts
  IkSampleableConstraint(
    const SampleablePoseConstraint& _isometry3dConstraint,
    const dart::dynamics::InverseKinematicsPtr& _ikPtr,
    std::unique_ptr<util::RNG> _rng,
    int _maxNumTrials);

  IkSampleableConstraint(const IkSampleableConstraint& other);
  IkSampleableConstraint(IkSampleableConstraint&& other);

  IkSampleableConstraint& operator=(
    const IkSampleableConstraint& other);
  IkSampleableConstraint& operator=(IkSampleableConstraint&& other);

  virtual ~IkSampleableConstraint() = default;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> 
    createSampleGenerator() const override;
  
  /// Set rng seed.
  void setRNG(std::unique_ptr<util::RNG> rng);
  
private:
  std::unique_ptr<util::RNG> mRng;
  SampleablePoseConstraint mIsometry3dConstraintPtr;
  dart::dynamics::InverseKinematicsPtr mIKPtr;
  int mMaxNumTrials;
};

using IkSampleableConstraintPtr = std::shared_ptr<const IkSampleableConstraint>;
using IkSampleableConstraintUniquePtr = std::unique_ptr<IkSampleableConstraint>;

} // namespace constraint 
} // namespace aikido

#endif // AIKIDO_Constraint_IKSAMPLEABLECONSTRAINT_H
