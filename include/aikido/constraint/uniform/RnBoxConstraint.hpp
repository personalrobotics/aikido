#ifndef AIKIDO_STATESPACE_RNBOXCONSTRAINT_HPP_
#define AIKIDO_STATESPACE_RNBOXCONSTRAINT_HPP_
#include "../../statespace/Rn.hpp"
#include "../Differentiable.hpp"
#include "../Projectable.hpp"
#include "../Sampleable.hpp"
#include "../Testable.hpp"

namespace aikido {
namespace constraint {

/// A BoxConstraint on RealVectorStates.
/// For each dimension, this constraint has lowerLimit and upperLimit.
template <int N>
class RBoxConstraint : public constraint::Differentiable,
                       public constraint::Projectable,
                       public constraint::Sampleable,
                       public constraint::Testable
{
public:
  using constraint::Projectable::project;
  using constraint::Differentiable::getValueAndJacobian;

  using VectorNd = Eigen::Matrix<double, N, 1>;

  /// Constructor.
  /// \param _space Space in which this constraint operates.
  /// \param _rng Random number generator to be used for sampling.
  /// \param _lowerLimits Lower limits on the states.
  ///        The length of this vector should match the dimension of _space.
  /// \param _upperLimits Upper limits.
  ///        The length of this vector should match the dimension of _space.
  RBoxConstraint(
      std::shared_ptr<statespace::R<N>> _space,
      std::unique_ptr<common::RNG> _rng,
      const VectorNd& _lowerLimits,
      const VectorNd& _upperLimits);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::size_t getConstraintDimension() const override;

  // Documentation inherited.
  std::vector<constraint::ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  bool isSatisfied(const statespace::StateSpace::State* state, bool _givenState = false) const override;

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
  std::shared_ptr<statespace::R<N>> mSpace;
  std::unique_ptr<common::RNG> mRng;
  VectorNd mLowerLimits;
  VectorNd mUpperLimits;
};

using R0BoxConstraint = RBoxConstraint<0>;
using R1BoxConstraint = RBoxConstraint<1>;
using R2BoxConstraint = RBoxConstraint<2>;
using R3BoxConstraint = RBoxConstraint<3>;
using R6BoxConstraint = RBoxConstraint<6>;
using RnBoxConstraint = RBoxConstraint<Eigen::Dynamic>;

} // namespace constraint
} // namespace aikido

#include "aikido/constraint/uniform/detail/RnBoxConstraint-impl.hpp"

#endif // AIKIDO_STATESPACE_REALVECTORSTATESPACESAMPLEABLECONSTRAINT_HPP_
