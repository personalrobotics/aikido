#ifndef AIKIDO_CONSTRAINT_CARTESIANPRODUCTSAMPLEABLE_HPP_
#define AIKIDO_CONSTRAINT_CARTESIANPRODUCTSAMPLEABLE_HPP_

#include <vector>

#include "aikido/constraint/Sampleable.hpp"
#include "aikido/statespace/CartesianProduct.hpp"

namespace aikido {
namespace constraint {

/// Sampleable for CompoundStates.
/// It takes in a set of Sampleables and its SampleGenerators
/// sample i-th substate from the i-th Sampleable.
class CartesianProductSampleable : public Sampleable
{
public:
  /// Constructor.
  /// \param _stateSpace StateSpace in which this constraint operates.
  /// \param _constraints Set of sampleables. The size of _constraints
  ///        should match the number of subspaces in _stateSpace.
  ///        i-th constraint applies to i-th subspace.
  CartesianProductSampleable(
      std::shared_ptr<const statespace::CartesianProduct> _stateSpace,
      std::vector<std::shared_ptr<Sampleable>> _constraints);

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  std::shared_ptr<const statespace::CartesianProduct> mStateSpace;
  std::vector<std::shared_ptr<Sampleable>> mConstraints;
};

} // namespace constraint
} // namespace aikido

#endif // define AIKIDO_CONSTRAINT_CARTESIANPRODUCTSAMPLEABLE_HPP_
