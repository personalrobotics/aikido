#ifndef AIKIDO_DISTANCE_CARTESIANPRODUCTWEIGHTED_HPP_
#define AIKIDO_DISTANCE_CARTESIANPRODUCTWEIGHTED_HPP_

#include "../statespace/CartesianProduct.hpp"
#include "DistanceMetric.hpp"

namespace aikido {
namespace distance {

/// Implements a distance metric on a CartesianProduct.
///
/// This metric computes the weighted sum of distances on the individual
/// components of the statespace.
class CartesianProductWeighted : public DistanceMetric
{
public:
  /// Constructor.
  ///
  /// Default the weights applied to each subspace to 1.
  /// \param _space The state space
  /// \param _metrics A vector containing one element for every component of the
  /// CartesianProduct
  CartesianProductWeighted(
      std::shared_ptr<statespace::CartesianProduct> _space,
      std::vector<DistanceMetricPtr> _metrics);

  /// Constructor.
  ///
  /// CartesianProduct. The first element of every pair in the vector is the
  /// metric and the second is the weight to be applied to the metric. The
  /// weights must all be positive.
  /// \param _space The state space
  /// \param _metrics A vector containing one element for every component of the
  CartesianProductWeighted(
      std::shared_ptr<statespace::CartesianProduct> _space,
      std::vector<std::pair<DistanceMetricPtr, double>> _metrics);

  // Documentation inherited
  statespace::ConstStateSpacePtr getStateSpace() const override;

  /// Computes distance between two states as the weighted sum of distances
  /// between their matching subcomponents.
  ///
  /// \param _state1 The first state (type CartesianProduct::State)
  /// \param _state2 The second state (type CartesianProduct::State)
  double distance(
      const statespace::StateSpace::State* _state1,
      const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<statespace::CartesianProduct> mStateSpace;
  std::vector<std::pair<DistanceMetricPtr, double>> mMetrics;
};

} // namespace distance
} // namespace aikido

#endif
