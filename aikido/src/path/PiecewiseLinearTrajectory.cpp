#include <aikido/path/PiecewiseLinearTrajectory.hpp>

namespace aikido {
namespace path {

using State = aikido::statespace::StateSpace::State;

PiecewiseLinearTrajectory::PiecewiseLinearTrajectory(
    const aikido::statespace::StateSpacePtr &_sspace,
    const aikido::distance::DistanceMetricPtr &_dmetric)
    : mStateSpace(_sspace), mDistanceMetric(_dmetric) {}

auto PiecewiseLinearTrajectory::getStateSpace() const
    -> aikido::statespace::StateSpacePtr {
  return mStateSpace;
}

int PiecewiseLinearTrajectory::getNumDerivatives() const { return 1; }

double PiecewiseLinearTrajectory::getFirstWaypointTime() const {
  if (mWaypoints.size() == 0)
    throw std::invalid_argument(
        "Requested getFirstWaypointTime on empty trajectory");
  return mWaypoints.front().first;
}

double PiecewiseLinearTrajectory::getLastWaypointTime() const {
  if (mWaypoints.size() == 0)
    throw std::invalid_argument(
        "Requested getLastWaypointTime on empty trajectory");
  return mWaypoints.back().first;
}

double PiecewiseLinearTrajectory::getDuration() const {
  if (mWaypoints.size() == 0)
    throw std::invalid_argument("Requested getDuration on empty trajectory");

  return getLastWaypointTime() - getFirstWaypointTime();
}

State *PiecewiseLinearTrajectory::evaluate(double _t) const {

  if (_t < getFirstWaypointTime() || _t > getLastWaypointTime()) {
    throw std::invalid_argument(
        "Requested trajectory point is outside the valid range");
  }

  auto it = std::lower_bound(mWaypoints.begin(), mWaypoints.end(), _t,
                             CompareWaypoints());
  auto idx = std::distance(mWaypoints.begin(), it);

  // TODO: Check validity

  Waypoint currentWpt = mWaypoints[idx];
  Waypoint prevWpt = mWaypoints[idx - 1];
  State *returnState = mStateSpace->allocateState();
  mDistanceMetric->interpolate(
      prevWpt.second, currentWpt.second,
      (_t - prevWpt.first) / (currentWpt.first - prevWpt.first), returnState);
  return returnState;
}

Eigen::VectorXd PiecewiseLinearTrajectory::evaluate(double _t,
                                                    int _derivative) const {
    if(_derivative == 0)
        throw std::invalid_argument("0th derivative not available. Use evaluate(t).");

    if(_derivative > 1)
        return Eigen::VectorXd::Zero(mStateSpace->getDimension());
}

void PiecewiseLinearTrajectory::addWaypoint(double _t, const State *_state) {

  State *state = mStateSpace->allocateState();
  mStateSpace->copyState(state, _state);

  // Maintain a sorted list of waypoints
  auto it = std::lower_bound(mWaypoints.begin(), mWaypoints.end(), _t,
                             CompareWaypoints());

  mWaypoints.insert(it, std::make_pair(_t, state));
}

// aikido::statespace::ScopedState
// PiecewiseLinearTrajectory::removeWaypoint(Scalar _t){

// }
}
}
