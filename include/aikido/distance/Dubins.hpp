#ifndef AIKIDO_DISTANCE_DUBINS_HPP_
#define AIKIDO_DISTANCE_DUBINS_HPP_

#include "aikido/planner/ompl/BackwardCompatibility.hpp"
#include "../statespace/SE2.hpp"
#include "DistanceMetric.hpp"

namespace aikido {
namespace distance {

/// Computes the shortest distance between two positions in SE2
/// when following a dubins path
class Dubins : public DistanceMetric
{
public:
  /// Constructor.
  /// \param _space The SE2 this distance metric operates on
  explicit Dubins(std::shared_ptr<statespace::SE2> _space);

  Dubins(std::shared_ptr<statespace::SE2> _space, double turningRadius, bool isSymmetric = false);

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
  double mTurningRadius;
  bool mIsSymmetric;
};

} // namespace distance
} // namespace aikido

#endif
