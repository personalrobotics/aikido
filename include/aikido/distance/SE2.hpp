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
  explicit SE2(std::shared_ptr<statespace::SE2> _space);

  // Documentation inherited
  statespace::ConstStateSpacePtr getStateSpace() const override;

  /// Computes shortest distance between two SE2 states.

  /// \param _state1 The first state (type: SE2::State)
  /// \param _state2 The second state (type: SE2::State)
  double distance(
      const statespace::StateSpace::State* _state1,
      const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<statespace::SE2> mStateSpace;
};

} // namespace distance
} // namespace aikido

#endif
