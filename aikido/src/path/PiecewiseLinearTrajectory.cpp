#include <aikido/path/PiecewiseLinearTrajectory.hpp>

namespace aikido
{
namespace path
{
using State = aikido::statespace::StateSpace::State;

PiecewiseLinearTrajectory::PiecewiseLinearTrajectory(
    const aikido::statespace::StateSpacePtr &_sspace,
    const aikido::distance::DistanceMetricPtr &_dmetric)
    : mStateSpace(_sspace)
    , mDistanceMetric(_dmetric)
{
}

auto PiecewiseLinearTrajectory::getStateSpace() const
    -> aikido::statespace::StateSpacePtr
{
  return mStateSpace;
}

//=============================================================================
int PiecewiseLinearTrajectory::getNumDerivatives() const { return 1; }

//=============================================================================
double PiecewiseLinearTrajectory::getFirstWaypointTime() const
{
  if (mWaypoints.size() == 0)
    throw std::invalid_argument(
        "Requested getFirstWaypointTime on empty trajectory");
  return mWaypoints.front().t;
}

//=============================================================================
double PiecewiseLinearTrajectory::getLastWaypointTime() const
{
  if (mWaypoints.size() == 0)
    throw std::invalid_argument(
        "Requested getLastWaypointTime on empty trajectory");
  return mWaypoints.back().t;
}

//=============================================================================
double PiecewiseLinearTrajectory::getDuration() const
{
  if (mWaypoints.size() == 0) return 0;

  return getLastWaypointTime() - getFirstWaypointTime();
}

//=============================================================================
void PiecewiseLinearTrajectory::evaluate(double _t, State *_state) const
{
  if (mWaypoints.size() == 0) {
    throw std::invalid_argument(
        "Requested trajectory point from an empty trajectory");
  }

  try {
    int idx = getWaypointIndexAfterTime(_t);
    if (idx == 0) {
      // Time before beginning of trajectory - return first waypoint
      mStateSpace->copyState(_state, mWaypoints[0].state);
    } else {
      Waypoint currentWpt = mWaypoints[idx];
      Waypoint prevWpt = mWaypoints[idx - 1];
      mDistanceMetric->interpolate(
          prevWpt.state, currentWpt.state,
          (_t - prevWpt.t) / (currentWpt.t - prevWpt.t), _state);
    }
  } catch (std::domain_error e) {
    // Time past end of trajectory - return last waypoint
    mStateSpace->copyState(_state, mWaypoints.back().state);
  }
}

//=============================================================================
Eigen::VectorXd PiecewiseLinearTrajectory::evaluate(double _t,
                                                    int _derivative) const
{
  if (_derivative == 0)
    throw std::invalid_argument(
        "0th derivative not available. Use evaluate(t, state).");

  if (_derivative > 1)
    return Eigen::VectorXd::Zero(mStateSpace->getDimension());

  // Compute the interpolated state at time t
  auto iPt = mStateSpace->createState();
  evaluate(_t, iPt);

  // Compute the endpoint of the segment
  try {
    int idx = getWaypointIndexAfterTime(_t);
    if (idx == 0) {
      // Time before beginning of trajector - return 0
      return Eigen::VectorXd::Zero(mStateSpace->getDimension());
    }
    State *endPt = mWaypoints[idx].state;

    // Compute segment time and distance
    double segmentTime = mWaypoints[idx].t - mWaypoints[idx - 1].t;
    double segmentLength = mDistanceMetric->distance(mWaypoints[idx - 1].state,
                                                     mWaypoints[idx].state);

    // iPt*exp(gamma*tangent) = endPt -> gamma*tangent = inv(iPt)*endPt
    auto invState = mStateSpace->createState();
    mStateSpace->getInverse(iPt, invState);

    auto cState = mStateSpace->createState();
    mStateSpace->compose(invState, endPt, cState);

    Eigen::VectorXd direction(mStateSpace->getDimension());
    mStateSpace->logMap(cState, direction);
    direction.normalize();

    return (segmentLength / segmentTime) * direction;
  } catch (std::domain_error e) {
    // Time past end of trajectory - return zero
    return Eigen::VectorXd::Zero(mStateSpace->getDimension());
  }
}

//=============================================================================
void PiecewiseLinearTrajectory::addWaypoint(double _t, const State *_state)
{
  State *state = mStateSpace->allocateState();
  mStateSpace->copyState(state, _state);

  // Maintain a sorted list of waypoints
  auto it = std::lower_bound(mWaypoints.begin(), mWaypoints.end(), _t);
  mWaypoints.insert(it, Waypoint(_t, state));
}

//=============================================================================
int PiecewiseLinearTrajectory::getWaypointIndexAfterTime(double _t) const
{
  auto it = std::lower_bound(mWaypoints.begin(), mWaypoints.end(), _t);
  if (it == mWaypoints.end()) {
    throw std::domain_error(
        "_t is larger than the time value on the last waypoint.");
  }

  return std::distance(mWaypoints.begin(), it);
}
}
}
