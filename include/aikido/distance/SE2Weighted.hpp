#ifndef AIKIDO_DISTANCE_SE2WEIGHTED_HPP_
#define AIKIDO_DISTANCE_SE2WEIGHTED_HPP_

#include "../statespace/SE2.hpp"
#include "DistanceMetric.hpp"

namespace aikido {
namespace distance {

/// Computes the shortest distance between two states in SE2
class SE2Weighted : public DistanceMetric
{
public:
  /// Constructor.
  ///
  /// \param space The SE2 this distance metric operates on
  /// The weights have been set to 1 as default
  explicit SE2Weighted(std::shared_ptr<const statespace::SE2> space);

  /// Constructor.
  ///
  /// It is a vector of 2 elements, the first element corresponds to the weight
  /// of translational distance and the second the weight for angular distance
  /// \param space The SE2 this distance metric operates on
  /// \param weights The weights over translational and angular distances
  SE2Weighted(
      std::shared_ptr<const statespace::SE2> space,
      const Eigen::Vector2d& weights);

  // Documentation inherited
  statespace::ConstStateSpacePtr getStateSpace() const override;

  /// Computes weighted distance between two SE2 states.
  ///
  /// \param state1 The first state (type: SE2::State)
  /// \param state2 The second state (type: SE2::State)
  double distance(
      const statespace::StateSpace::State* state1,
      const statespace::StateSpace::State* state2) const override;

private:
  std::shared_ptr<const statespace::SE2> mStateSpace;

  Eigen::Vector2d mWeights;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_SE2WEIGHTED_HPP_
