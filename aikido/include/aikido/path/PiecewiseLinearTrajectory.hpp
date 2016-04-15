#ifndef AIKIDO_PATH_PIECEWISELINEAR_TRAJECTORY_H_
#define AIKIDO_PATH_PIECEWISELINEAR_TRAJECTORY_H_

#include "Trajectory.hpp"
#include "../distance/DistanceMetric.hpp"
#include "../statespace/GeodesicInterpolator.hpp"

namespace aikido
{
namespace path
{
/// Implements a piecewise linear trajectory
class PiecewiseLinearTrajectory : public Trajectory
{
public:
  explicit PiecewiseLinearTrajectory(aikido::statespace::StateSpacePtr _sspace);

  // Documentation inherited
  aikido::statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited
  int getNumDerivatives() const override;

  /// The time on the first waypoint of the trajectory
  double getStartTime() const;

  /// The time on the last waypoint of the trajectory
  double getEndTime() const;

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
  std::shared_ptr<aikido::statespace::GeodesicInterpolator> mInterpolator;
  std::vector<Waypoint> mWaypoints;
};
}
}

#endif
