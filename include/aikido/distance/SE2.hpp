#ifndef AIKIDO_DISTANCE_SE2DISTANCEMETRIC_HPP_
#define AIKIDO_DISTANCE_SE2DISTANCEMETRIC_HPP_

#include "../statespace/SE2.hpp"
#include "DistanceMetric.hpp"

namespace aikido {
namespace distance {

/// Computes the shortest distance between two angles in SE2
class SE2 : public DistanceMetric
{
public:

  /// Constructor.
  /// \param _space The SE2 this distance metric operates on
  /// The weights have been set to 1 as default
  SE2(std::shared_ptr<statespace::SE2> _space);

  /// Constructor.
  /// \param _space The SE2 this distance metric operates on
  /// \param _weights The weights over angular and translational distances
  /// It is a vector of 2 elements, the first element corresponds to the weight
  /// of angular distance and the second the weight for translational distance
  SE2(
    std::shared_ptr<statespace::SE2> _space,
    Eigen::Vector2d _weights);

  // Documentation inherited
  statespace::StateSpacePtr getStateSpace() const override;

  /// Computes weighted distance between two SE2 states.

  /// \param _state1 The first state (type: SE2::State)
  /// \param _state2 The second state (type: SE2::State)
  double distance(
      const statespace::StateSpace::State* _state1,
      const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<statespace::SE2> mStateSpace;
  Eigen::Vector2d mWeights;
};

} // namespace distance
} // namespace aikido

#endif
