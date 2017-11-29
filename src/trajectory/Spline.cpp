#include <aikido/trajectory/Spline.hpp>

#include <dart/common/common.hpp>
#include <aikido/common/Spline.hpp>

namespace aikido {
namespace trajectory {

//==============================================================================
Spline::Spline(statespace::StateSpacePtr _stateSpace, double _startTime)
  : mStateSpace(std::move(_stateSpace)), mStartTime(_startTime), mSegments()
{
  if (mStateSpace == nullptr)
    throw std::invalid_argument("StateSpace is null.");
}

//==============================================================================
Spline::Spline(const Spline& other)
  : mStateSpace(other.mStateSpace), mStartTime(other.mStartTime), mSegments()
{
  for (const auto& segment : other.mSegments)
    addSegment(segment.mCoefficients, segment.mDuration, segment.mStartState);
}

//==============================================================================
Spline::Spline(Spline&& other)
  : mStateSpace(std::move(other.mStateSpace))
  , mStartTime(std::move(other.mStartTime))
  , mSegments(std::move(other.mSegments))
{
  // Do nothing
}

//==============================================================================
Spline::~Spline()
{
  for (const auto& segment : mSegments)
    mStateSpace->freeState(segment.mStartState);
}

//==============================================================================
Spline& Spline::operator=(const Spline& other)
{
  Spline newSpline(other);
  *this = std::move(newSpline);

  return *this;
}

//==============================================================================
Spline& Spline::operator=(Spline&& other)
{
  mStateSpace = std::move(other.mStateSpace);
  mStartTime = std::move(other.mStartTime);
  mSegments = std::move(other.mSegments);

  return *this;
}

//==============================================================================
std::unique_ptr<Trajectory> Spline::clone() const
{
  return dart::common::make_unique<Spline>(*this);
}

//==============================================================================
void Spline::addSegment(
    const Eigen::MatrixXd& _coefficients,
    double _duration,
    const statespace::StateSpace::State* _startState)
{
  if (_duration <= 0.)
    throw std::invalid_argument("Duration must be positive.");

  if (static_cast<std::size_t>(_coefficients.rows())
      != mStateSpace->getDimension())
    throw std::invalid_argument("Incorrect number of dimensions.");

  if (_coefficients.cols() < 1)
    throw std::invalid_argument("At least one coefficient is required.");

  PolynomialSegment segment;
  segment.mCoefficients = _coefficients;
  segment.mDuration = _duration;
  segment.mStartState = mStateSpace->allocateState();
  mStateSpace->copyState(_startState, segment.mStartState);

  mSegments.emplace_back(std::move(segment));
}

//==============================================================================
void Spline::addSegment(const Eigen::MatrixXd& _coefficients, double _duration)
{
  if (mSegments.empty())
  {
    throw std::logic_error(
        "An explicit start state is required because this trajectory is "
        "empty.");
  }

  auto startState = mStateSpace->createState();
  evaluate(getEndTime(), startState);

  addSegment(_coefficients, _duration, startState);
}

//==============================================================================
std::size_t Spline::getNumSegments() const
{
  return mSegments.size();
}

//==============================================================================
void Spline::setSegmentDuration(std::size_t index, double duration)
{
  if (index >= getNumSegments())
    throw std::domain_error("Segment index is out of bounds.");

  if (duration <= 0.0)
    throw std::domain_error("Segment duration should be positive value.");

  mSegments[index].mDuration = duration;
}

//==============================================================================
double Spline::getSegmentDuration(std::size_t index) const
{
  if (index >= getNumSegments())
    throw std::domain_error("Segment index is out of bounds.");

  return mSegments[index].mDuration;
}

//==============================================================================
void Spline::setSegmentCoefficients(
    std::size_t index, const Eigen::MatrixXd& coefficients)
{
  if (index >= getNumSegments())
    throw std::domain_error("Segment index is out of bounds.");

  mSegments[index].mCoefficients = coefficients;
}

//==============================================================================
void Spline::setSegmentCoefficients(
    std::size_t index, const Eigen::Map<const Eigen::MatrixXd>& coefficients)
{
  if (index >= getNumSegments())
    throw std::domain_error("Segment index is out of bounds.");

  mSegments[index].mCoefficients = coefficients;
}

//==============================================================================
void Spline::setSegmentCoefficient(
    std::size_t segmentIndex,
    std::size_t coeffRow,
    std::size_t coeffCol,
    double coefficient)
{
  if (segmentIndex >= getNumSegments())
    throw std::domain_error("Segment index is out of bounds.");

  if (coeffRow
      >= static_cast<std::size_t>(mSegments[segmentIndex].mCoefficients.rows()))
  {
    throw std::domain_error("Spline coefficient's index is out of bounds.");
  }

  if (coeffCol
      >= static_cast<std::size_t>(mSegments[segmentIndex].mCoefficients.cols()))
  {
    throw std::domain_error("Spline coefficient's index is out of bounds.");
  }

  mSegments[segmentIndex].mCoefficients(coeffRow, coeffCol) = coefficient;
}

//==============================================================================
const Eigen::MatrixXd& Spline::getSegmentCoefficients(std::size_t index) const
{
  if (index >= getNumSegments())
    throw std::domain_error("Segment index is out of bounds.");

  return mSegments[index].mCoefficients;
}

//==============================================================================
statespace::StateSpacePtr Spline::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::size_t Spline::getNumDerivatives() const
{
  std::size_t numDerivatives = 0;

  for (const auto& segment : mSegments)
  {
    numDerivatives = std::max<std::size_t>(
        numDerivatives, segment.mCoefficients.cols() - 1);
  }

  return numDerivatives;
}

//==============================================================================
double Spline::getStartTime() const
{
  return mStartTime;
}

//==============================================================================
double Spline::getEndTime() const
{
  return mStartTime + getDuration();
}

//==============================================================================
double Spline::getDuration() const
{
  double duration = 0.;

  for (const auto& segment : mSegments)
    duration += segment.mDuration;

  return duration;
}

//==============================================================================
void Spline::evaluate(double _t, statespace::StateSpace::State* _out) const
{
  if (mSegments.empty())
    throw std::logic_error("Unable to evaluate empty trajectory.");

  const auto targetSegmentInfo = getSegmentForTime(_t);
  const auto& targetSegment = mSegments[targetSegmentInfo.first];

  mStateSpace->copyState(targetSegment.mStartState, _out);

  const auto evaluationTime = _t - targetSegmentInfo.second;
  const auto tangentVector
      = evaluatePolynomial(targetSegment.mCoefficients, evaluationTime, 0);

  const auto relativeState = mStateSpace->createState();
  mStateSpace->expMap(tangentVector, relativeState);
  mStateSpace->compose(_out, relativeState);
}

//==============================================================================
void Spline::evaluateDerivative(
    double _t, int _derivative, Eigen::VectorXd& _tangentVector) const
{
  if (mSegments.empty())
    throw std::logic_error("Unable to evaluate empty trajectory.");
  if (_derivative < 1)
    throw std::logic_error("Derivative must be positive.");

  const auto targetSegmentInfo = getSegmentForTime(_t);
  const auto& targetSegment = mSegments[targetSegmentInfo.first];
  const auto evaluationTime = _t - targetSegmentInfo.second;

  // Return zero for higher-order derivatives.
  if (_derivative < targetSegment.mCoefficients.cols())
  {
    // TODO: We should transform this into the body frame using the adjoint
    // transformation.
    _tangentVector = evaluatePolynomial(
        targetSegment.mCoefficients, evaluationTime, _derivative);
  }
  else
  {
    _tangentVector.resize(mStateSpace->getDimension());
    _tangentVector.setZero();
  }
}

//==============================================================================
std::pair<std::size_t, double> Spline::getSegmentForTime(double _t) const
{
  auto segmentStartTime = mStartTime;

  for (std::size_t isegment = 0; isegment < mSegments.size(); ++isegment)
  {
    const auto& segment = mSegments[isegment];
    const auto nextSegmentStartTime = segmentStartTime + segment.mDuration;

    if (_t <= nextSegmentStartTime)
      return std::make_pair(isegment, segmentStartTime);

    segmentStartTime = nextSegmentStartTime;
  }

  // After the end of the last segment.
  return std::make_pair(
      mSegments.size() - 1, segmentStartTime - mSegments.back().mDuration);
}

//==============================================================================
Eigen::VectorXd Spline::evaluatePolynomial(
    const Eigen::MatrixXd& _coefficients, double _t, int _derivative)
{
  const auto numOutputs = _coefficients.rows();
  const auto numCoeffs = _coefficients.cols();

  const auto timeVector
      = common::SplineProblem<>::createTimeVector(_t, _derivative, numCoeffs);
  const auto derivativeMatrix
      = common::SplineProblem<>::createCoefficientMatrix(numCoeffs);
  const auto derivativeVector = derivativeMatrix.row(_derivative);
  const auto evaluationVector
      = derivativeVector.cwiseProduct(timeVector.transpose());

  Eigen::VectorXd outputVector(numOutputs);

  for (int ioutput = 0; ioutput < numOutputs; ++ioutput)
    outputVector[ioutput] = _coefficients.row(ioutput).dot(evaluationVector);

  return outputVector;
}

//==============================================================================
std::size_t Spline::getNumWaypoints() const
{
  return getNumSegments() + 1;
}

//==============================================================================
double Spline::getWaypointTime(std::size_t _index) const
{
  double waypointTime = mStartTime;

  if (_index >= getNumWaypoints())
    throw std::domain_error("Waypoint index is out of bounds.");

  for (std::size_t i = 0; i < _index; ++i)
    waypointTime += mSegments[i].mDuration;

  return waypointTime;
}

//==============================================================================
void Spline::getWaypoint(
    std::size_t _index, statespace::StateSpace::State* state) const
{
  if (_index < getNumWaypoints())
  {
    double waypointTime = getWaypointTime(_index);
    evaluate(waypointTime, state);
  }
  else
  {
    throw std::domain_error("Waypoint index is out of bounds.");
  }
}

//==============================================================================
void Spline::getWaypointDerivative(
    std::size_t _index, int _derivative, Eigen::VectorXd& _tangentVector) const
{
  if (_index < getNumWaypoints())
  {
    double waypointTime = getWaypointTime(_index);
    evaluateDerivative(waypointTime, _derivative, _tangentVector);
  }
  else
  {
    throw std::domain_error("Waypoint index is out of bounds.");
  }
}

} // namespace trajectory
} // namespace aikido
