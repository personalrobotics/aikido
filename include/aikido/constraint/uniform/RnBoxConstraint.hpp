#ifndef AIKIDO_CONSTRAINT_UNIFORM_RNBOXCONSTRAINT_HPP_
#define AIKIDO_CONSTRAINT_UNIFORM_RNBOXCONSTRAINT_HPP_

#include "aikido/constraint/Differentiable.hpp"
#include "aikido/constraint/Projectable.hpp"
#include "aikido/constraint/Sampleable.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/statespace/Rn.hpp"

namespace aikido {
namespace constraint {
namespace uniform {

/// A BoxConstraint on RealVectorStates.
/// For each dimension, this constraint has lowerLimit and upperLimit.
template <int N>
class RBoxConstraint : public constraint::Differentiable,
                       public constraint::Projectable,
                       public constraint::Sampleable,
                       public constraint::Testable
{
public:
  using constraint::Differentiable::getValueAndJacobian;
  using constraint::Projectable::project;

  using VectorNd = Eigen::Matrix<double, N, 1>;

  /// Constructor.
  /// \param _space Space in which this constraint operates.
  /// \param _rng Random number generator to be used for sampling.
  /// \param _lowerLimits Lower limits on the states.
  ///        The length of this vector should match the dimension of _space.
  /// \param _upperLimits Upper limits.
  ///        The length of this vector should match the dimension of _space.
  RBoxConstraint(
      std::shared_ptr<const statespace::R<N>> _space,
      std::unique_ptr<common::RNG> _rng,
      const VectorNd& _lowerLimits,
      const VectorNd& _upperLimits);

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::size_t getConstraintDimension() const override;

  // Documentation inherited.
  std::vector<constraint::ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  bool isSatisfied(
      const statespace::StateSpace::State* state,
      TestableOutcome* outcome = nullptr) const override;

  /// Return an instance of DefaultTestableOutcome, since this class doesn't
  /// have a more specialized TestableOutcome derivative assigned to it.
  std::unique_ptr<TestableOutcome> createOutcome() const override;

  // Documentation inherited.
  bool project(
      const statespace::StateSpace::State* _s,
      statespace::StateSpace::State* _out) const override;

  // Documentation inherited.
  void getValue(const statespace::StateSpace::State* _s, Eigen::VectorXd& _out)
      const override;

  // Documentation inherited.
  void getJacobian(
      const statespace::StateSpace::State* _s,
      Eigen::MatrixXd& _out) const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator> createSampleGenerator()
      const override;

  /// Returns lower limits of this constraint.
  auto getLowerLimits() const -> const VectorNd&;

  /// Returns upper limits of this constraint.
  auto getUpperLimits() const -> const VectorNd&;

private:
  std::shared_ptr<const statespace::R<N>> mSpace;
  std::unique_ptr<common::RNG> mRng;
  VectorNd mLowerLimits;
  VectorNd mUpperLimits;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(VectorNd::NeedsToAlign)
};

using R0BoxConstraint = RBoxConstraint<0>;
using R1BoxConstraint = RBoxConstraint<1>;
using R2BoxConstraint = RBoxConstraint<2>;
using R3BoxConstraint = RBoxConstraint<3>;
using R6BoxConstraint = RBoxConstraint<6>;
using RnBoxConstraint = RBoxConstraint<Eigen::Dynamic>;

} // namespace uniform
} // namespace constraint
} // namespace aikido

#include "aikido/constraint/uniform/detail/RnBoxConstraint-impl.hpp"

#endif // AIKIDO_CONSTRAINT_UNIFORM_RNBOXCONSTRAINT_HPP_
