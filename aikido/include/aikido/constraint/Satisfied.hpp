#ifndef AIKIDO_CONSTRAINT_SATISFIED_HPP_
#define AIKIDO_CONSTRAINT_SATISFIED_HPP_
#include "Differentiable.hpp"
#include "Projectable.hpp"
#include "Sampleable.hpp"
#include "Testable.hpp"

namespace aikido {
namespace constraint {

/// A constraint which is always satisfied.
/// This class is often used in CartesianProduct constraints 
/// to represent that some subspace doesn't have any constraint.
class Satisfied 
  : public constraint::Differentiable
  , public constraint::Projectable
  , public constraint::Testable
{
public:
  /// Constructor.
  /// \param _space StateSpace in which this constraint operates.
  explicit Satisfied(statespace::StateSpacePtr _space);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Returns 0.
  size_t getConstraintDimension() const override;

  /// Returns an empty vector.
  std::vector<constraint::ConstraintType> getConstraintTypes() const override;

  /// Returns true.
  bool isSatisfied(const statespace::StateSpace::State* state) const override;

  /// _out will always match the value of _s.
  bool project(
    const statespace::StateSpace::State* _s,
    statespace::StateSpace::State* _out) const override;

  // Returns an empty vector.
  Eigen::VectorXd getValue(
    const statespace::StateSpace::State* _s) const override;

  /// Returns an empty matrix.
  Eigen::MatrixXd getJacobian(
    const statespace::StateSpace::State* _s) const override;

  /// Returns a pair of empty vector and empty matrix.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
    const statespace::StateSpace::State* _s) const override;

private:
  statespace::StateSpacePtr mStateSpace;
};

} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_SATISFIED_H_
