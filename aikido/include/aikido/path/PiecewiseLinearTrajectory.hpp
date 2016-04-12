#ifndef AIKIDO_PATH_PIECEWISELINEAR_TRAJECTORY_H_
#define AIKIDO_PATH_PIECEWISELINEAR_TRAJECTORY_H_

#include "Trajectory.hpp"
#include "../distance/DistanceMetric.hpp"

namespace aikido {
namespace path {

/// Implements a piecewise linear trajectory
class PiecewiseLinearTrajectory : public Trajectory {
public:
  // Pair defining a waypint in the trajectory
  typedef std::pair<double, aikido::statespace::StateSpace::State *> Waypoint;

  PiecewiseLinearTrajectory(
      const aikido::statespace::StateSpacePtr &_sspace,
      const aikido::distance::DistanceMetricPtr &_dmetric);

  // Documentation inherited
  aikido::statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited
  int getNumDerivatives() const override;

  /// The time on the first waypoint of the trajectory
  double getFirstWaypointTime() const;

  /// The time on the last waypoint of the trajectory
  double getLastWaypointTime() const;

  // Documentation inherited
  double getDuration() const override;

  // Documentation inherited
  aikido::statespace::StateSpace::State *evaluate(double _t) const override;

  // Documentation inherited
  Eigen::VectorXd evaluate(double _t, int _derivative) const override;

  /// Add a waypoint to the trajectory at the given time
  /// The State is copied into the trajectory
  void addWaypoint(double _t,
                   const aikido::statespace::StateSpace::State *_state);

  /// Remove and return the State at the given Waypoint
  ///  This removes the waypoint from the trajectory.
  ///  The timing on the remaining waypoints remainds unaffected.
  // aikido::statespace::ScopedState removeWaypoint(Scalar _t);

private:
  aikido::statespace::StateSpacePtr mStateSpace;
  aikido::distance::DistanceMetricPtr mDistanceMetric;
  std::vector<Waypoint> mWaypoints;
};
}
}

#include "detail/PiecewiseLinearTrajectory.hpp"

#endif
