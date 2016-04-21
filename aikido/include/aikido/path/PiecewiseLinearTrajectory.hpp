#ifndef AIKIDO_PATH_PIECEWISELINEAR_TRAJECTORY_H_
#define AIKIDO_PATH_PIECEWISELINEAR_TRAJECTORY_H_

#include "Trajectory.hpp"
#include "../statespace/GeodesicInterpolator.hpp"

namespace aikido {
namespace path {

/// Implements a piecewise linear trajectory
class PiecewiseLinearTrajectory : public Trajectory
{
public:
  PiecewiseLinearTrajectory(
    aikido::statespace::StateSpacePtr _sspace,
    aikido::statespace::InterpolatorPtr _interpolator);

  // Documentation inherited
  aikido::statespace::StateSpacePtr getStateSpace() const override;

  /// Gets the interpolator used to interpolate between waypoints.
  aikido::statespace::InterpolatorPtr getInterpolator() const;

  // Documentation inherited
  int getNumDerivatives() const override;

  // Documentation inherited
  double getStartTime() const override;

  // Documentation inherited
  double getEndTime() const override;

  // Documentation inherited
  double getDuration() const override;

  // Documentation inherited
  void evaluate(double _t,
                aikido::statespace::StateSpace::State *_state) const override;

  // Documentation inherited
  Eigen::VectorXd evaluate(double _t, int _derivative) const override;

  /// Add a waypoint to the trajectory at the given time
  /// The State is copied into the trajectory
  void addWaypoint(double _t,
                   const aikido::statespace::StateSpace::State *_state);

  /// Gets the i-th waypoint.
  const statespace::StateSpace::State* getWaypoint(size_t _index) const;

  /// Gets the number of waypoints.
  size_t getNumWaypoints() const;

private:
  // Pair defining a waypint in the trajectory
  struct Waypoint {
    Waypoint(double _t, aikido::statespace::StateSpace::State *_state)
        : t(_t)
        , state(_state)
    {
    }

    // The time associated with this waypoint
    double t;

    // The state associated with this waypoint
    aikido::statespace::StateSpace::State *state;

    /// Comparator to allow sorting waypoints based on time
    bool operator<(const Waypoint &rhs) const { return t < rhs.t; }

    /// Comparator to allow sorting waypoints based on time
    bool operator<(const double &rhs) const { return t < rhs; }
  };

  /// Get the index of the first waypoint whose
  ///  time value is larger than _t.
  /// Throws std::domain_error if _t is larger
  /// than last waypoint in the trajectory
  int getWaypointIndexAfterTime(double _t) const;

  aikido::statespace::StateSpacePtr mStateSpace;
  aikido::statespace::InterpolatorPtr mInterpolator;
  std::vector<Waypoint> mWaypoints;
};

using PiecewiseLinearTrajectoryPtr = std::shared_ptr<PiecewiseLinearTrajectory>;

} // namespace path
} // namespace aikido

#endif
