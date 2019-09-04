#ifndef AIKIDO_CONSTRAINT_CARTESIANPRODUCTPROJECTABLE_HPP_
#define AIKIDO_CONSTRAINT_CARTESIANPRODUCTPROJECTABLE_HPP_

#include "aikido/statespace/CartesianProduct.hpp"
#include "Projectable.hpp"

namespace aikido {
namespace constraint {

/// A Projectable for CompoundStates.
/// It takes in a set of Projectables and project
/// i-th substate to i-th constraint.
class CartesianProductProjectable : public Projectable
{
public:
  /// Constructor.
  /// \param _stateSpace CartesianProduct in which this constraint operates.
  /// \param _constraints Set of constraints. The size of _constraints
  ///        should match the number of subspaces in _stateSpace.
  ///        i-th constraint applies to i-th subspace.
  CartesianProductProjectable(
      std::shared_ptr<const statespace::CartesianProduct> _stateSpace,
      std::vector<ProjectablePtr> _constraints);

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  bool project(
      const statespace::StateSpace::State* _s,
      statespace::StateSpace::State* _out) const override;

private:
  std::shared_ptr<const statespace::CartesianProduct> mStateSpace;
  std::vector<ProjectablePtr> mConstraints;
};

} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_CARTESIANPRODUCTPROJECTABLE_HPP_
