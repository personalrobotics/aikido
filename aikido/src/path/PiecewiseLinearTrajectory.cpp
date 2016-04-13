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
  if (mWaypoints.size() == 0)
    throw std::invalid_argument("Requested getDuration on empty trajectory");

  return getLastWaypointTime() - getFirstWaypointTime();
}

//=============================================================================
State *PiecewiseLinearTrajectory::evaluate(double _t) const
{
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
      prevWpt.state, currentWpt.state,
      (_t - prevWpt.t) / (currentWpt.t - prevWpt.t), returnState);
  return returnState;
}

//=============================================================================
Eigen::VectorXd PiecewiseLinearTrajectory::evaluate(double _t,
                                                    int _derivative) const
{
  if (_derivative == 0)
    throw std::invalid_argument(
        "0th derivative not available. Use evaluate(t).");

  if (_derivative > 1)
    return Eigen::VectorXd::Zero(mStateSpace->getDimension());

  // Compute the point at d
  State *iPt = evaluate(_t);

  // Compute the endpoint of the segment
  auto it = std::lower_bound(mWaypoints.begin(), mWaypoints.end(), _t,
                             CompareWaypoints());
  auto idx = std::distance(mWaypoints.begin(), it);
  State *endPt = mWaypoints[idx].state;  // TODO: bounds check

  // Compute segment time and distance
  double segmentTime = mWaypoints[idx].t - mWaypoints[idx - 1].t;
  double segmentLength =
      mDistanceMetric->distance(mWaypoints[idx-1].state, mWaypoints[idx].state);

  // iPt*exp(gamma*tangent) = endPt -> gamma*tangent = inv(iPt)*endPt
  auto invState = mStateSpace->createState();
  mStateSpace->getInverse(iPt, invState);

  auto cState = mStateSpace->createState();
  mStateSpace->compose(invState, endPt, cState);

  Eigen::VectorXd direction(mStateSpace->getDimension());
  mStateSpace->logMap(cState, direction);
  direction.normalize();

  return (segmentLength / segmentTime) * direction;
}

//=============================================================================
void PiecewiseLinearTrajectory::addWaypoint(double _t, const State *_state)
{
  State *state = mStateSpace->allocateState();
  mStateSpace->copyState(state, _state);

  // Maintain a sorted list of waypoints
  auto it = std::lower_bound(mWaypoints.begin(), mWaypoints.end(), _t,
                             CompareWaypoints());

  mWaypoints.insert(it, Waypoint(_t, state));
}
}
}
