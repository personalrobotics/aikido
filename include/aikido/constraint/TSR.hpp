#ifndef AIKIDO_CONSTRAINT_TSR_HPP_
#define AIKIDO_CONSTRAINT_TSR_HPP_

#include <Eigen/Dense>
#include <dart/math/MathTypes.hpp>
#include "../statespace/SE3.hpp"
#include "Differentiable.hpp"
#include "Projectable.hpp"
#include "Sampleable.hpp"
#include "Testable.hpp"

namespace aikido {
namespace constraint {

/// TSRs describe end-effector constraint sets as subsets of SE(3).
/// A TSR consists of three parts:
///     T0_w: transform from the origin to the TSR frame w
///     B_w: 6 × 2 matrix of bounds in the coordinates of w.
///     Tw_e: end-effector offset transform in the coordinates of w
/// See:
/// Berenson, Dmitry, Siddhartha S. Srinivasa, and James Kuffner.
/// "Task space regions: A framework for pose-constrained manipulation
/// planning." IJRR 2001:
/// http://repository.cmu.edu/cgi/viewcontent.cgi?article=2024&context=robotics
class TSR : public Sampleable,
            public Differentiable,
            public Testable,
            public Projectable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Differentiable::getValueAndJacobian;

  /// Constructor.
  /// \param _rng Random number generator used by SampleGenerators for this TSR.
  /// \param _T0_w transform from the origin to the TSR frame w
  /// \param _Bw 6 × 2 matrix of bounds in the coordinates of w.
  ///        Top three rows bound translation, and bottom three rows
  ///        bound rotation following Roll-Pitch-Yaw convention.
  ///        _Bw(i, 0) should be less than or equal to _Bw(i, 1).
  /// \param _Tw_e end-effector offset transform in the coordinates of w
  /// \param _testableTolerance tolerance used in isSatisfiable as testable
  TSR(std::unique_ptr<common::RNG> _rng,
      const Eigen::Isometry3d& _T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& _Bw
      = Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& _Tw_e = Eigen::Isometry3d::Identity(),
      double _testableTolerance = 1e-6);

  /// Constructor with default random seed generator.
  /// \param _T0_w transform from the origin to the TSR frame w
  /// \param _Bw 6 × 2 matrix of bounds in the coordinates of w.
  ///        Top three rows bound translation, and bottom three rows
  ///        bound rotation following Roll-Pitch-Yaw convention.
  ///        _Bw(i, 0) should be less than or equal to _Bw(i, 1).
  /// \param _Tw_e end-effector offset transform in the coordinates of w
  /// \param _testableTolerance tolerance used in isSatisfiable as testable
  TSR(const Eigen::Isometry3d& _T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& _Bw
      = Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& _Tw_e = Eigen::Isometry3d::Identity(),
      double _testableTolerance = 1e-6);

  TSR(const TSR& other);
  TSR(TSR&& other);

  TSR& operator=(const TSR& other);
  TSR& operator=(TSR&& other);

  virtual ~TSR() = default;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Returns the SE3 which this TSR operates in.
  std::shared_ptr<statespace::SE3> getSE3() const;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

  // Documentation inherited.
  bool isSatisfied(
      const statespace::StateSpace::State* _s,
      TestableOutcome* outcome = nullptr) const override;

  /// Return an instance of DefaultOutcome, since this class doesn't have a
  /// more specialized TestableOutcome derivative assigned to it.
  std::unique_ptr<TestableOutcome> createOutcome() const override;

  /// Throws an invalid_argument exception if this TSR is invalid.
  /// For a TSR to be valid, mBw(i, 0) <= mBw(i, 1).
  void validate() const;

  /// Set the random number generator used by SampleGenerators for this TSR.
  void setRNG(std::unique_ptr<common::RNG> rng);

  // Documentation inherited.
  std::size_t getConstraintDimension() const override;

  // Documentation inherited.
  void getValue(const statespace::StateSpace::State* _s, Eigen::VectorXd& _out)
      const override;

  /// Jacobian of TSR with respect to the se(3) tangent vector of _s.
  /// The jacobian is w.r.t. the origin frame.
  /// se(3) tangent vector follows dart convention:
  ///   top 3 rows is the angle-axis representation of _s's rotation.
  ///   bottom 3 rows represent the translation.
  /// \param _s State to be evaluated at.
  /// \param[out] _out Jacobian, 6 x 6 matrix.
  void getJacobian(
      const statespace::StateSpace::State* _s,
      Eigen::MatrixXd& _out) const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  bool project(
      const statespace::StateSpace::State* _s,
      statespace::StateSpace::State* _out) const override;

  /// Get the testable tolerance used in isSatisfiable.
  /// \param[out] _out Testable tolerance, double.
  double getTestableTolerance();

  /// Set the testable tolerance used in isSatisfiable.
  /// \param _testableTolerance Testable tolerance to set.
  void setTestableTolerance(double _testableTolerance);

  /// Transformation from origin frame into the TSR frame "w".
  /// "w" is usually centered at the origin of an object held by the hand
  ///  or at a location on an object that is useful for grasping.
  Eigen::Isometry3d mT0_w;

  /// Bounds on "wiggling" in `x, y, z, roll, pitch, yaw`.
  Eigen::Matrix<double, 6, 2> mBw;

  /// Transformation from "w" frame into end frame.
  /// This often represent an offset from "w" to the origin of the end-effector.
  Eigen::Isometry3d mTw_e;

private:
  /// Tolerance used in isSatisfied as a testable
  double mTestableTolerance;
  std::unique_ptr<common::RNG> mRng;
  std::shared_ptr<statespace::SE3> mStateSpace;
};

using TSRPtr = std::shared_ptr<TSR>;

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_TSR_HPP_
