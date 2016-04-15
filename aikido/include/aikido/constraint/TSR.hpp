#ifndef AIKIDO_CONSTRAINT_TSR_H_
#define AIKIDO_CONSTRAINT_TSR_H_

#include "Sampleable.hpp"
#include "../statespace/SE3StateSpace.hpp"
#include <Eigen/Dense>
#include "Projectable.hpp"
#include "Differentiable.hpp"
#include "TestableConstraint.hpp"
#include <dart/math/MathTypes.h>

namespace aikido
{
namespace constraint
{
class TSR : public SampleableConstraint,
            public Differentiable,
            public TestableConstraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TSR(std::unique_ptr<util::RNG> _rng,
      const Eigen::Isometry3d& _T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& _Bw =
          Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& _Tw_e = Eigen::Isometry3d::Identity());

  TSR(const Eigen::Isometry3d& _T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& _Bw =
          Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& _Tw_e = Eigen::Isometry3d::Identity());

  TSR(const TSR& other);
  TSR(TSR&& other);

  TSR& operator=(const TSR& other);
  TSR& operator=(TSR&& other);

  virtual ~TSR() = default;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::shared_ptr<statespace::SE3StateSpace> getSE3StateSpace() const;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

  // Documentation inherited.
  bool isSatisfied(const statespace::StateSpace::State* _s) const override;

  /// Throws an invalid_argument exception if this TSR is invalid.
  void validate() const;

  /// Set the random number generator used by SampleGenerators for this TSR.
  void setRNG(std::unique_ptr<util::RNG> rng);

  // Documentation inherited.
  size_t getConstraintDimension() const override;

  // Documentation inherited.
  Eigen::VectorXd getValue(
      const statespace::StateSpace::State* _s) const override;

  // Returns 6 x 6 matrix.
  // Jacobian of TSR w.r.t. se3 tangent vector in World Frame
  // -- the frame SE3 was written w.r.t.
  Eigen::MatrixXd getJacobian(
      const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
      const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  /// Transformation from origin frame into "wiggle" frame.
  Eigen::Isometry3d mT0_w;

  /// Bounds on "wiggling" in `x, y, z, roll, pitch, yaw`.
  Eigen::Matrix<double, 6, 2> mBw;

  /// Transformation from "wiggle" frame into end frame.
  Eigen::Isometry3d mTw_e;

private:
  std::unique_ptr<util::RNG> mRng;
  std::shared_ptr<statespace::SE3StateSpace> mStateSpace;
};

using TSRPtr = std::shared_ptr<TSR>;

}  // namespace constraint
}  // namespace aikido

#endif  // AIKIDO_CONSTRAINT_TSR_H_
