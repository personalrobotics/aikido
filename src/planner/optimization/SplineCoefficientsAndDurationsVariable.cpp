#include "aikido/planner/optimization/SplineCoefficientsAndDurationsVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
SplineCoefficientsAndDurationsVariable::SplineCoefficientsAndDurationsVariable(
    const trajectory::Spline& splineToClone)
  : SplineVariable(splineToClone)
{
  updateDimension();
}

//==============================================================================
std::unique_ptr<Variable> SplineCoefficientsAndDurationsVariable::clone() const
{
  return dart::common::make_unique<SplineCoefficientsAndDurationsVariable>(
      *this);
}

//==============================================================================
void SplineCoefficientsAndDurationsVariable::setValue(
    const Eigen::VectorXd& value)
{
  if (static_cast<std::size_t>(value.size()) != getDimension())
    throw std::invalid_argument("Invalid size of value given");

  const auto rows = mSpline.getStateSpace()->getDimension();
  int segmentIndex = mSpline.getNumSegments();
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    const auto cols = mSpline.getSegmentCoefficients(i).cols();
    const auto numLocalVariables = rows * cols;

    Eigen::Map<const Eigen::MatrixXd> newSegmentCoeffis(
        value.segment(segmentIndex, numLocalVariables).data(), rows, cols);
    mSpline.setSegmentCoefficients(i, newSegmentCoeffis);
    mSpline.setSegmentDuration(i, value[i]);

    segmentIndex += numLocalVariables;
  }

  assert(static_cast<std::size_t>(segmentIndex) == getDimension());
}

//==============================================================================
Eigen::VectorXd SplineCoefficientsAndDurationsVariable::getValue() const
{
  Eigen::VectorXd value(getDimension());

  const auto rows = mSpline.getStateSpace()->getDimension();
  int segmentIndex = mSpline.getNumSegments();
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    auto& segmentCoeffs = mSpline.getSegmentCoefficients(i);

    const auto cols = segmentCoeffs.cols();
    const auto numLocalVariables = rows * cols;

    value.segment(segmentIndex, numLocalVariables)
        = Eigen::Map<const Eigen::VectorXd>(
            segmentCoeffs.data(), numLocalVariables);
    value[i] = mSpline.getSegmentDuration(i);

    segmentIndex += numLocalVariables;
  }

  assert(static_cast<std::size_t>(segmentIndex) == getDimension());
  return value;
}

//==============================================================================
void SplineCoefficientsAndDurationsVariable::setCoefficientValueTo(
    Eigen::VectorXd& vector, double value) const
{
  if (static_cast<std::size_t>(vector.size()) != getDimension())
    throw std::invalid_argument("Invalid size of vector given");

  vector.tail(getDimension() - mSpline.getNumSegments()).setConstant(value);
}

//==============================================================================
void SplineCoefficientsAndDurationsVariable::setCoefficientValueTo(
    Eigen::VectorXd& vector, const Eigen::VectorXd& values) const
{
  if (static_cast<std::size_t>(vector.size()) != getDimension())
    throw std::invalid_argument("Invalid size of vector given");

  const auto rows = mSpline.getStateSpace()->getDimension();

  if (static_cast<std::size_t>(values.size()) != rows)
    throw std::invalid_argument("Invalid size of vector given");

  int segmentIndex = mSpline.getNumSegments();
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    const auto cols = mSpline.getSegmentCoefficients(i).cols();

    for (auto j = 0; j < cols; ++j)
    {
      vector.segment(segmentIndex, rows) = values;
      segmentIndex += rows;
    }
  }

  assert(static_cast<std::size_t>(segmentIndex) == getDimension());
}

//==============================================================================
void SplineCoefficientsAndDurationsVariable::setDurationValueTo(
    Eigen::VectorXd& vector, double duration)
{
  vector.head(mSpline.getNumSegments()).setConstant(duration);
}

//==============================================================================
void SplineCoefficientsAndDurationsVariable::setDurationValueTo(
    Eigen::VectorXd& vector, const Eigen::VectorXd& duration)
{
  if (static_cast<std::size_t>(vector.size()) != getDimension())
    throw std::invalid_argument("Invalid size of vector given");

  if (static_cast<std::size_t>(duration.size()) != mSpline.getNumSegments())
    throw std::invalid_argument("Invalid duration vector");

  vector.head(mSpline.getNumSegments()) = duration;
}

//==============================================================================
void SplineCoefficientsAndDurationsVariable::setDurationValueTo(
    Eigen::VectorXd& vector, std::size_t segmentIndex, double duration)
{
  if (static_cast<std::size_t>(vector.size()) != getDimension())
    throw std::invalid_argument("Invalid size of vector given");

  // TODO(JS): Check validity of segmentIndex

  vector[segmentIndex] = duration;
}

//==============================================================================
void SplineCoefficientsAndDurationsVariable::updateDimension()
{
  std::size_t dim = 0u;

  const auto& statespace = mSpline.getStateSpace();

  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    const auto& segmentCoeffs = mSpline.getSegmentCoefficients(i);

    // This should be guaranteed by Spline::addSegment(). Here we just check
    // this for sure.
    assert(
        static_cast<std::size_t>(segmentCoeffs.rows())
        == statespace->getDimension());

    dim += segmentCoeffs.cols();
  }

  dim *= statespace->getDimension();
  dim += mSpline.getNumSegments(); // for durations

  mDimension = dim;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
