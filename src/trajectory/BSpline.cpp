#include "aikido/trajectory/BSpline.hpp"

#include <sstream>
#include <Eigen/Core>
#include <dart/common/common.hpp>
#include "aikido/common/StepSequence.hpp"

namespace aikido {
namespace trajectory {

namespace {

//==============================================================================
void throwIfInvalidIndex(
    const std::vector<BSpline::SplineType>& splines, std::size_t index)
{
  if (index >= splines.size())
  {
    std::stringstream ss;
    ss << "The index '" << index << "' is out of the range [0, "
       << splines.size() << "].\n";
    throw std::invalid_argument(ss.str());
  }
}

} // (anonymous) namespace

//==============================================================================
BSpline::BSpline(
    statespace::ConstStateSpacePtr stateSpace,
    const BSpline::KnotVectorType& knots,
    const BSpline::ControlPointVectorType& controlPoints)
  : mStateSpace(std::move(stateSpace))
{
  if (mStateSpace == nullptr)
  {
    throw std::invalid_argument("StateSpace is null.");
  }

  if (knots.size() < 3)
  {
    std::stringstream ss;
    ss << "Number of knots '" << knots.size() << "' should be at least two.";
    throw std::invalid_argument(ss.str());
  }

  if (controlPoints.size() < 1)
  {
    std::stringstream ss;
    ss << "Number of control points '" << knots.size()
       << "' should be at least one.";
    throw std::invalid_argument(ss.str());
  }

  if (knots.size() - controlPoints.size() - 1 < 0)
  {
    std::stringstream ss;
    ss << "Degree '" << knots.size() - controlPoints.size() - 1
       << "' (# of knots ' " << knots.size() << "' - # of control points '"
       << controlPoints.size() << "' - 1) should be at least zero.";
    throw std::invalid_argument(ss.str());
  }

  mStartTime = knots[0];
  mEndTime = knots[knots.size() - 1];

  mSplines.reserve(mStateSpace->getDimension());
  for (auto i = 0u; i < mStateSpace->getDimension(); ++i)
    mSplines.emplace_back(knots, controlPoints);
}

//==============================================================================
BSpline::BSpline(
    statespace::ConstStateSpacePtr stateSpace,
    std::size_t degree,
    const BSpline::ControlPointVectorType& controlPoints,
    double startTime,
    double endTime)
  : BSpline(
        std::move(stateSpace),
        computeUniformKnots(
            degree,
            static_cast<std::size_t>(controlPoints.size()),
            startTime,
            endTime),
        controlPoints)
{
  if (static_cast<std::size_t>(controlPoints.size()) <= degree)
  {
    std::stringstream ss;
    ss << "The number of control points '" << controlPoints.size()
       << "' should be greater than the degree '" << degree << "'.";
    throw std::invalid_argument(ss.str());
  }
}

//==============================================================================
BSpline::BSpline(
    statespace::ConstStateSpacePtr stateSpace,
    std::size_t degree,
    std::size_t numControlPoints,
    double startTime,
    double endTime)
  : BSpline(
        std::move(stateSpace),
        degree,
        ControlPointVectorType::Zero(
            static_cast<ControlPointVectorType::Index>(numControlPoints)),
        startTime,
        endTime)
{
  if (startTime >= endTime)
  {
    std::stringstream ss;
    ss << "The start-time '" << startTime << "' should be less than the "
       << "end-time '" << endTime << "'.";
    throw std::invalid_argument(ss.str());
  }
}

//==============================================================================
BSpline::BSpline(const BSpline& other)
  : mStateSpace(other.mStateSpace)
  , mSplines(other.mSplines)
  , mStartTime(other.mStartTime)
  , mEndTime(other.mEndTime)
{
  // Do nothing
}

//==============================================================================
BSpline::BSpline(BSpline&& other)
  : mStateSpace(std::move(other.mStateSpace))
  , mSplines(std::move(other.mSplines))
  , mStartTime(std::move(other.mStartTime))
  , mEndTime(std::move(other.mEndTime))
{
  // Do nothing
}

//==============================================================================
BSpline::~BSpline()
{
  // Do nothing
}

//==============================================================================
BSpline& BSpline::operator=(const BSpline& other)
{
  BSpline newBSpline(other);
  *this = std::move(newBSpline);

  return *this;
}

//==============================================================================
BSpline& BSpline::operator=(BSpline&& other)
{
  mStateSpace = std::move(other.mStateSpace);
  mSplines = std::move(other.mSplines);
  mStartTime = std::move(other.mStartTime);
  mEndTime = std::move(other.mEndTime);

  return *this;
}

//==============================================================================
std::unique_ptr<Trajectory> BSpline::clone() const
{
  return ::dart::common::make_unique<BSpline>(*this);
}

//==============================================================================
std::size_t BSpline::getDegree() const
{
  if (mStateSpace->getDimension() == 0u)
    return 0; // TODO: Better idea?

  assert(!mSplines.empty());

  return static_cast<std::size_t>(mSplines[0].degree());
}

//==============================================================================
std::size_t BSpline::getOrder() const
{
  return getDegree() + 1u;
}

//==============================================================================
std::size_t BSpline::getNumKnots() const
{
  if (mStateSpace->getDimension() == 0u)
    return 0; // TODO: Better idea?

  assert(!mSplines.empty());

  return static_cast<std::size_t>(mSplines[0].knots().size());
}

//==============================================================================
std::size_t BSpline::getNumControlPoints() const
{
  if (mStateSpace->getDimension() == 0u)
    return 0; // TODO: Better idea?

  assert(!mSplines.empty());

  return static_cast<std::size_t>(mSplines[0].ctrls().size());
}

//==============================================================================
void BSpline::setStartPoint(std::size_t stateSpaceIndex, double value)
{
  throwIfInvalidIndex(mSplines, stateSpaceIndex);

  mSplines[stateSpaceIndex].ctrls()[0] = value;
}

//==============================================================================
void BSpline::setStartPoint(const Eigen::VectorXd& point)
{
  const auto stateSpaceDim = mStateSpace->getDimension();

  if (stateSpaceDim != static_cast<std::size_t>(point.size()))
    throw std::invalid_argument("Invalid dimension");

  for (auto i = 0u; i < stateSpaceDim; ++i)
    mSplines[i].ctrls()[0] = point[i];
}

//==============================================================================
void BSpline::setStartPoint(const statespace::StateSpace::State* state)
{
  Eigen::VectorXd values;
  mStateSpace->logMap(state, values);
  setStartPoint(values);
}

//==============================================================================
void BSpline::setEndPoint(std::size_t stateSpaceIndex, double value)
{
  throwIfInvalidIndex(mSplines, stateSpaceIndex);

  const std::size_t numCtrPts = getNumControlPoints();

  if (numCtrPts == 0)
    return;

  mSplines[stateSpaceIndex].ctrls()[static_cast<int>(numCtrPts) - 1] = value;
}

//==============================================================================
void BSpline::setEndPoint(const Eigen::VectorXd& point)
{
  const std::size_t numCtrPts = getNumControlPoints();

  if (numCtrPts == 0)
    return;

  const auto stateSpaceDim = mStateSpace->getDimension();

  if (stateSpaceDim != static_cast<std::size_t>(point.size()))
    throw std::invalid_argument("Invalid dimension");

  for (auto i = 0u; i < stateSpaceDim; ++i)
    mSplines[i].ctrls()[static_cast<int>(numCtrPts) - 1] = point[i];
}

//==============================================================================
void BSpline::setEndPoint(const statespace::StateSpace::State* state)
{
  if (getNumControlPoints() == 0)
    return;

  Eigen::VectorXd values;
  mStateSpace->logMap(state, values);
  setEndPoint(values);
}

//==============================================================================
void BSpline::setControlPoints(
    std::size_t stateSpaceIndex,
    const BSpline::ControlPointVectorType& controlPoints,
    bool withStartControlPoint,
    bool withEndControlPoint)
{
  throwIfInvalidIndex(mSplines, stateSpaceIndex);

  const auto numControlPoints
      = static_cast<ControlPointVectorType::Index>(getNumControlPoints());

  if (numControlPoints == 0)
    return;

  if (withStartControlPoint)
  {
    if (withEndControlPoint)
    {
      if (controlPoints.size() != numControlPoints)
        throw std::invalid_argument("Invalid number of control points");

      mSplines[stateSpaceIndex].ctrls() = controlPoints;
    }
    else
    {
      if (controlPoints.size() - 1 != numControlPoints)
        throw std::invalid_argument("Invalid number of control points");

      mSplines[stateSpaceIndex].ctrls().head(numControlPoints - 1u)
          = controlPoints;
    }
  }
  else
  {
    if (withEndControlPoint)
    {
      if (controlPoints.size() - 1 != numControlPoints)
        throw std::invalid_argument("Invalid number of control points");

      mSplines[stateSpaceIndex].ctrls().tail(numControlPoints - 1u)
          = controlPoints;
    }
    else
    {
      if (controlPoints.size() - 2 != numControlPoints)
        throw std::invalid_argument("Invalid number of control points");

      mSplines[stateSpaceIndex].ctrls().segment(1, numControlPoints - 2u)
          = controlPoints;
    }
  }
}

//==============================================================================
void BSpline::setControlPoints(
    std::size_t stateSpaceIndex,
    double value,
    bool withStartControlPoint,
    bool withEndControlPoint)
{
  throwIfInvalidIndex(mSplines, stateSpaceIndex);

  const auto numControlPoints
      = static_cast<ControlPointVectorType::Index>(getNumControlPoints());

  if (numControlPoints == 0)
    return;

  if (withStartControlPoint)
  {
    if (withEndControlPoint)
    {
      mSplines[stateSpaceIndex].ctrls().setConstant(value);
    }
    else
    {
      mSplines[stateSpaceIndex]
          .ctrls()
          .head(numControlPoints - 1)
          .setConstant(value);
    }
  }
  else
  {
    if (withEndControlPoint)
    {
      mSplines[stateSpaceIndex]
          .ctrls()
          .tail(numControlPoints - 1)
          .setConstant(value);
    }
    else
    {
      mSplines[stateSpaceIndex]
          .ctrls()
          .segment(1, numControlPoints - 2)
          .setConstant(value);
    }
  }
}

//==============================================================================
const BSpline::ControlPointVectorType& BSpline::getControlPoints(
    std::size_t stateSpaceIndex) const
{
  throwIfInvalidIndex(mSplines, stateSpaceIndex);

  return mSplines[stateSpaceIndex].ctrls();
}

//==============================================================================
BSpline::ControlPointVectorType BSpline::getControlPoints(
    std::size_t stateSpaceIndex,
    bool withStartControlPoint,
    bool withEndControlPoint) const
{
  throwIfInvalidIndex(mSplines, stateSpaceIndex);

  const auto numControlPoints
      = static_cast<ControlPointVectorType::Index>(getNumControlPoints());

  if (withStartControlPoint)
  {
    if (withEndControlPoint)
      return getControlPoints(stateSpaceIndex);
    else
      return mSplines[stateSpaceIndex].ctrls().head(numControlPoints - 1);
  }
  else
  {
    if (withEndControlPoint)
      return mSplines[stateSpaceIndex].ctrls().tail(numControlPoints - 1);
    else
      return mSplines[stateSpaceIndex].ctrls().segment(1, numControlPoints - 2);
  }
}

//==============================================================================
statespace::ConstStateSpacePtr BSpline::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::size_t BSpline::getNumDerivatives() const
{
  return getDegree();
}

//==============================================================================
double BSpline::getStartTime() const
{
  return mStartTime;
}

//==============================================================================
double BSpline::getEndTime() const
{
  return mEndTime;
}

//==============================================================================
double BSpline::getDuration() const
{
  return mEndTime - mStartTime;
}

//==============================================================================
void throwIfInvalidTime(const BSpline& spline, double t)
{
  const auto startTime = spline.getStartTime();
  const auto endTime = spline.getEndTime();

  if (t < startTime || endTime < t)
  {
    std::stringstream ss;
    ss << "The evaluating time '" << t << "' is out of the duration ["
       << startTime << ", " << endTime << "], which isn't allowed.";
    throw std::invalid_argument(ss.str());
  }
}

//==============================================================================
void BSpline::evaluate(double t, statespace::StateSpace::State* state) const
{
  throwIfInvalidTime(*this, t);

  Eigen::VectorXd values(mStateSpace->getDimension());

  for (auto i = 0u; i < mStateSpace->getDimension(); ++i)
    values[i] = mSplines[i](t)[0];

  mStateSpace->expMap(values, state);
}

//==============================================================================
void BSpline::evaluateDerivative(
    double t, int derivative, Eigen::VectorXd& /*tangentVector*/) const
{
  throwIfInvalidTime(*this, t);

  if (derivative < 1)
    throw std::invalid_argument("Derivative must be positive.");

  // TODO(JS): Not implemented
}

//==============================================================================
double BSpline::computeArcLength(
    const distance::DistanceMetric& distanceMetric, double resolution) const
{
  const common::StepSequence steps(
      resolution, true, true, getStartTime(), getEndTime());

  if (steps.getLength() == 0u)
    return 0.0;

  auto state0 = mStateSpace->createState();
  auto state1 = mStateSpace->createState();

  double arcLength = 0.0;

  auto it = steps.begin();
  evaluate(*it, state0);
  for (; it != steps.end(); ++it)
  {
    evaluate(*it, state1);
    arcLength += distanceMetric.distance(state0, state1);

    mStateSpace->copyState(state1, state0);
  }

  return arcLength;
}

//==============================================================================
std::size_t BSpline::computeNumKnots(
    std::size_t degree, std::size_t numControlPoints)
{
  return degree + numControlPoints + 1u;
}

//==============================================================================
BSpline::KnotVectorType BSpline::computeUniformKnots(
    std::size_t degree,
    std::size_t numControlPoints,
    double startTime,
    double endTime)
{
  if (numControlPoints <= degree)
  {
    throw std::invalid_argument(
        "Number of control points should be greater than degree");
  }

  const auto numKnots = computeNumKnots(degree, numControlPoints);

  KnotVectorType knots(numKnots);
  knots.head(static_cast<ControlPointVectorType::Index>(degree))
      .setConstant(startTime);
  knots.tail(static_cast<ControlPointVectorType::Index>(degree))
      .setConstant(endTime);

  const common::StepSequence internalKnots(
      startTime, endTime, numControlPoints + 1u - degree, true);
  ControlPointVectorType::Index index = static_cast<int>(degree);
  assert(
      degree + internalKnots.getLength() + degree
      == static_cast<std::size_t>(knots.size()));
  for (double knot : internalKnots)
    knots[index++] = knot;

  return knots;
}

} // namespace trajectory
} // namespace aikido
