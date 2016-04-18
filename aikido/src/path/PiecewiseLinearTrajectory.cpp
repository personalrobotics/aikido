#include <aikido/path/PiecewiseLinearTrajectory.hpp>

using aikido::statespace::GeodesicInterpolator;

namespace aikido
{
namespace path
{

using State = aikido::statespace::StateSpace::State;

//=============================================================================
PiecewiseLinearTrajectory::PiecewiseLinearTrajectory(
      aikido::statespace::StateSpacePtr _sspace,
      aikido::statespace::InterpolatorPtr _interpolator)
  : mStateSpace(std::move(_sspace))
  , mInterpolator(std::move(_interpolator))
{
}

//=============================================================================
statespace::StateSpacePtr PiecewiseLinearTrajectory::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
statespace::InterpolatorPtr PiecewiseLinearTrajectory::getInterpolator() const
{
  return mInterpolator;
}

//=============================================================================
int PiecewiseLinearTrajectory::getNumDerivatives() const
{
  return mInterpolator->getNumDerivatives();
}

//=============================================================================
double PiecewiseLinearTrajectory::getStartTime() const
{
  if (mWaypoints.empty())
    throw std::domain_error("Requested getEndTime on empty trajectory.");

  return mWaypoints.front().t;
}

//=============================================================================
double PiecewiseLinearTrajectory::getEndTime() const
{
  if (mWaypoints.empty())
    throw std::domain_error("Requested getEndTime on empty trajectory.");

  return mWaypoints.back().t;
}

//=============================================================================
double PiecewiseLinearTrajectory::getDuration() const
{
  if (!mWaypoints.empty())
    return getEndTime() - getStartTime();
  else
    return 0.;
}

//=============================================================================
void PiecewiseLinearTrajectory::evaluate(double _t, State *_state) const
{
  if (mWaypoints.empty())
    throw std::invalid_argument(
        "Requested trajectory point from an empty trajectory");

  try {
    int idx = getWaypointIndexAfterTime(_t);
    if (idx == 0) {
      // Time before beginning of trajectory - return first waypoint
      mStateSpace->copyState(_state, mWaypoints[0].state);
    } else {
      Waypoint currentWpt = mWaypoints[idx];
      Waypoint prevWpt = mWaypoints[idx - 1];
      mInterpolator->interpolate(
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

  if (_derivative > mInterpolator->getNumDerivatives())
    return Eigen::VectorXd::Zero(mStateSpace->getDimension());

  // Compute the interpolated state at time t
  auto iPt = mStateSpace->createState();
  evaluate(_t, iPt);

  // Compute the endpoint of the segment
  try
  {
    int idx = getWaypointIndexAfterTime(_t);

    // Time before beginning of trajectory - return 0
    if (idx == 0)
      return Eigen::VectorXd::Zero(mStateSpace->getDimension());

    const auto segmentTime = mWaypoints[idx].t - mWaypoints[idx - 1].t;
    const auto alpha = (_t - mWaypoints[idx - 1].t) / segmentTime;
    const auto tangentVector = mInterpolator->getDerivative(
      mWaypoints[idx - 1].state, mWaypoints[idx].state, _derivative, alpha);

    return tangentVector / segmentTime;
  }
  catch (const std::domain_error& e)
  {
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
const statespace::StateSpace::State* PiecewiseLinearTrajectory::getWaypoint(
  size_t _index) const
{
  if (_index < mWaypoints.size())
    return mWaypoints[_index].state;
  else
    throw std::domain_error("Waypoint index is out of bounds.");
}

//=============================================================================
size_t PiecewiseLinearTrajectory::getNumWaypoints() const
{
  return mWaypoints.size();
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
