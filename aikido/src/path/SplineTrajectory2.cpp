#include <aikido/path/Spline.hpp>
#include <aikido/path/SplineTrajectory2.hpp>

namespace aikido {
namespace path {

//=============================================================================
SplineTrajectory2::SplineTrajectory2(
      statespace::StateSpacePtr _stateSpace,
      const statespace::StateSpace::State* _startState,
      double _startTime,
      std::vector<PolynomialSegment> _segments)
  : mStateSpace(std::move(_stateSpace))
  , mStartState() // Initialized below.
  , mStartTime(_startTime)
  , mSegments(std::move(_segments))
{
  if (mSegments.empty())
    throw std::invalid_argument("At least one segment is required.");

  const auto numDimensions = _stateSpace->getDimension();

  for (size_t isegment = 0; isegment < mSegments.size(); ++isegment)
  {
    auto& segment = mSegments[isegment];

    if (segment.mDuration <= 0.)
    {
      std::stringstream msg;
      msg << "Segment " << isegment << " does not have a positive duration: "
          << segment.mDuration << ".";
      throw std::invalid_argument(msg.str());
    }
    if (segment.mCoefficients.cols() != numDimensions)
    {
      std::stringstream msg;
      msg << "Segment " << isegment << " has incorrect number of dimensions: "
          << segment.mCoefficients.cols() << " != " << numDimensions << ".";
      throw std::invalid_argument(msg.str());
    }
    if (segment.mCoefficients.rows() == 0)
    {
      std::stringstream msg;
      msg << "Segment " << isegment << " has zero coefficients.";
      throw std::invalid_argument(msg.str());
    }
  }

  mStartState = mStateSpace->allocateState();
  mStateSpace->copyState(mStartState, _startState);
}

//=============================================================================
SplineTrajectory2::~SplineTrajectory2()
{
  mStateSpace->freeState(mStartState);
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
      numDerivatives, segment.mCoefficients.rows() - 1);
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
  double _t, statespace::StateSpace::State *_state) const
{
  if (_t < mStartTime)
    throw std::domain_error("Time is before start time.");

  auto timeFromStart = mStartTime;
  auto previousState = mStateSpace->createState();
  auto relativeState = mStateSpace->createState();
  auto nextState = mStateSpace->createState();

  mStateSpace->copyState(previousState, mStartState);

  for (const auto& segment : mSegments)
  {
    const auto nextTimeFromStart = timeFromStart + segment.mDuration;

    if (nextTimeFromStart < _t)
    {
      const auto tangentVector = evaluatePolynomial(
        segment.mCoefficients, nextTimeFromStart, 0);
      mStateSpace->expMap(tangentVector, relativeState);
      mStateSpace->compose(previousState, relativeState, nextState);

      // TODO: This could be done efficiently through pointer-swapping.
      mStateSpace->copyState(previousState, nextState);
    }
    else
    {
      const auto tangentVector = evaluatePolynomial(
        segment.mCoefficients, _t, 0);
      mStateSpace->expMap(tangentVector, relativeState);
      mStateSpace->compose(previousState, relativeState, _state);
      return;
    }

    // TODO: Forward-integrate previousState.
    timeFromStart = nextTimeFromStart;
  }

  // Output the last waypoint of the trajectory.
  mStateSpace->copyState(_state, nextState);
}

//=============================================================================
Eigen::VectorXd SplineTrajectory2::evaluate(double _t, int _derivative) const
{
  throw std::runtime_error("not implemented");
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
  const auto evaluationVector = derivativeVector.cwiseProduct(timeVector);

  Eigen::VectorXd outputVector(numOutputs);

  for (size_t ioutput = 0; ioutput < numOutputs; ++ioutput)
    outputVector[ioutput] = _coefficients.row(ioutput).dot(evaluationVector);

  return outputVector;
}

} // namespace path
} // namespace aikido
