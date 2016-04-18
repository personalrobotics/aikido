#include <aikido/path/Spline.hpp>
#include <aikido/path/SplineTrajectory2.hpp>

namespace aikido {
namespace path {

//=============================================================================
SplineTrajectory2::SplineTrajectory2(
      statespace::StateSpacePtr _stateSpace,
      const statespace::StateSpace::State* _startState,
      double _startTime)
  : mStateSpace(std::move(_stateSpace))
  , mStartState() // Initialized below.
  , mStartTime(_startTime)
{
  if (mStateSpace == nullptr)
    throw std::invalid_argument("StateSpace is null.");

  if (_startState == nullptr)
    throw std::invalid_argument("Start state is null.");

  // Do this last, since we have to clean up this memory in the destructor.
  mStartState = mStateSpace->allocateState();
  mStateSpace->copyState(mStartState, _startState);
}

//=============================================================================
SplineTrajectory2::~SplineTrajectory2()
{
  mStateSpace->freeState(mStartState);
}

//=============================================================================
void SplineTrajectory2::addSegment(
  const Eigen::MatrixXd& _coefficients, double _duration)
{
  if (_duration <= 0.)
    throw std::invalid_argument("Duration must be positive.");

  if (_coefficients.rows() != mStateSpace->getDimension())
    throw std::invalid_argument("Incorrect number of dimensions.");

  if (_coefficients.cols() < 1)
    throw std::invalid_argument("At least one coefficient is required.");

  PolynomialSegment segment;
  segment.mCoefficients = _coefficients;
  segment.mDuration = _duration;

  mSegments.emplace_back(std::move(segment));
}

//=============================================================================
size_t SplineTrajectory2::getNumSegments() const
{
  return mSegments.size();
}

//=============================================================================
statespace::StateSpacePtr SplineTrajectory2::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
int SplineTrajectory2::getNumDerivatives() const
{
  int numDerivatives = 0;

  for (const auto& segment : mSegments)
  {
    numDerivatives = std::max<int>(
      numDerivatives, segment.mCoefficients.cols() - 1);
  }

  return numDerivatives;
}

//=============================================================================
double SplineTrajectory2::getStartTime() const
{
  return mStartTime;
}

//=============================================================================
double SplineTrajectory2::getEndTime() const
{
  return mStartTime + getDuration();
}

//=============================================================================
double SplineTrajectory2::getDuration() const
{
  double duration = 0.;

  for (const auto& segment : mSegments)
    duration += segment.mDuration;

  return duration;
}

//=============================================================================
void SplineTrajectory2::evaluate(
  double _t, statespace::StateSpace::State *_out) const
{
  if (mSegments.empty())
  {
    mStateSpace->copyState(_out, mStartState);
    return;
  }

  const auto targetSegmentInfo = getSegmentForTime(_t);
  const auto& targetSegment = mSegments[targetSegmentInfo.first];

  mStateSpace->copyState(_out, mStartState);

  const auto relativeState = mStateSpace->createState();
  const auto nextState = mStateSpace->createState();

  for (size_t isegment = 0; isegment <= targetSegmentInfo.first; ++isegment)
  {
    const auto& segment = mSegments[isegment];

    double evaluationTime;
    if (isegment < targetSegmentInfo.first)
      evaluationTime = segment.mDuration; // end of the segment
    else
      evaluationTime = _t - targetSegmentInfo.second; // target time

    const auto tangentVector = evaluatePolynomial(
      segment.mCoefficients, evaluationTime, 0);

    mStateSpace->expMap(tangentVector, relativeState);
    mStateSpace->compose(_out, relativeState, nextState);
    mStateSpace->copyState(_out, nextState);
  }
}

//=============================================================================
Eigen::VectorXd SplineTrajectory2::evaluate(double _t, int _derivative) const
{
  // Returns zero for an empty trajectory.
  if (mSegments.empty())
    return Eigen::VectorXd::Zero(mStateSpace->getDimension());

  const auto targetSegmentInfo = getSegmentForTime(_t);
  const auto& targetSegment = mSegments[targetSegmentInfo.first];
  const auto evaluationTime = _t - targetSegmentInfo.second;

  // Return zero for higher-order derivatives.
  if (_derivative < targetSegment.mCoefficients.cols())
    return evaluatePolynomial(targetSegment.mCoefficients, evaluationTime,
      _derivative);
  else
    return Eigen::VectorXd::Zero(mStateSpace->getDimension());
}

//=============================================================================
std::pair<size_t, double> SplineTrajectory2::getSegmentForTime(double _t) const
{
  auto segmentStartTime = mStartTime;

  for (size_t isegment = 0; isegment < mSegments.size(); ++isegment)
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

//=============================================================================
void SplineTrajectory2::getSegmentStartState(
  size_t _index, statespace::StateSpace::State* _out) const
{
  assert(_index < mSegments.size());
  assert(_out != nullptr);

  mStateSpace->copyState(_out, mStartState);

  const auto relativeState = mStateSpace->createState();
  const auto nextState = mStateSpace->createState();

  // Forward-integrate all previous segments.
  for (size_t isegment = 0; isegment < _index; ++isegment)
  {
    const auto& segment = mSegments[isegment];
    // TODO: Should we be using relative or absolute time here?
    const auto tangentVector = evaluatePolynomial(
      segment.mCoefficients, segment.mDuration, 0);

    // Compute the relative state offset caused by this segment.
    mStateSpace->expMap(tangentVector, relativeState);
    mStateSpace->compose(_out, relativeState, nextState);
    mStateSpace->copyState(_out, nextState);
  }
}

//=============================================================================
Eigen::VectorXd SplineTrajectory2::evaluatePolynomial(
  const Eigen::MatrixXd& _coefficients, double _t, int _derivative)
{
  const auto numOutputs = _coefficients.rows();
  const auto numCoeffs = _coefficients.cols();

  const auto timeVector = SplineProblem<>::createTimeVector(
    _t, _derivative, numCoeffs);
  const auto derivativeMatrix = SplineProblem<>::createCoefficientMatrix(
    numCoeffs);
  const auto derivativeVector = derivativeMatrix.row(_derivative);
  const auto evaluationVector = derivativeVector.cwiseProduct(timeVector.transpose());

  Eigen::VectorXd outputVector(numOutputs);

  for (size_t ioutput = 0; ioutput < numOutputs; ++ioutput)
    outputVector[ioutput] = _coefficients.row(ioutput).dot(evaluationVector);

  return outputVector;
}

} // namespace path
} // namespace aikido
