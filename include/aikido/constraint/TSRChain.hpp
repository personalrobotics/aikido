#ifndef AIKIDO_CONSTRAINT_TSRCHAIN_H_
#define AIKIDO_CONSTRAINT_TSRCHAIN_H_

#include <vector>
#include <Eigen/Dense>
#include <dart/math/MathTypes.hpp>
#include <aikido/statespace/SE3.hpp>
#include "Sampleable.hpp"
#include "Differentiable.hpp"
#include "Testable.hpp"
#include "TSR.hpp"


namespace aikido {
namespace constraint {

enum class TSRChainConstraintType {NONE, START_ONLY, GOAL_ONLY, 
                                   BOTH_START_GOAL, TRAJECTORY_WIDE};

///  A TSR chain is a combination of TSRs representing a motion constraint.
///  TSR chains compose multiple TSRs and the conditions under which they
///  must hold.  This class provides support for start, goal, and/or
///  trajectory-wide constraints.  They can be constructed from one or more
///  TSRs which must be applied together.
class TSRChain : public Sampleable,
                 public Differentiable,
                 public Testable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Constructor.
  TSRChain(const TSRChainConstraintType& type = TSRChainConstraintType::NONE,
           const std::vector<TSR>& tsrs = std::vector<TSR>() );

  TSRChain(const TSRChain& other);
  TSRChain(TSRChain&& other);

  TSRChain& operator=(const TSRChain& other);
  TSRChain& operator=(TSRChain&& other);

  virtual ~TSRChain() = default;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Returns the SE3 which this TSR chain operates in.
  std::shared_ptr<statespace::SE3> getSE3() const;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

  // Documentation inherited.
  bool isSatisfied(const statespace::StateSpace::State* _s) const override;

  /// Throws an invalid_argument exception if this TSR Chain is invalid.
  void validate() const;

  // Documentation inherited.
  size_t getConstraintDimension() const override;

  // Documentation inherited.
  void getValue(
    const statespace::StateSpace::State* _s,
    Eigen::VectorXd& _out) const override;

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

  /// Append a new TSR to TSR Chain.
  void append(const TSR& tsr);

  /// A chain of TSRs.
  std::vector<TSR> mTSRs;

  /// TSR Constraint Type.
  TSRChainConstraintType mTSRChainConstraintType;

private:
  std::shared_ptr<statespace::SE3> mStateSpace;
};

using TSRChainPtr = std::shared_ptr<TSRChain>;

}  // namespace constraint
}  // namespace aikido

#endif  // AIKIDO_CONSTRAINT_TSRCHAIN_H_
