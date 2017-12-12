#include "aikido/planner/optimization/SplineCoefficientsAndDurationsVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
SplineCoefficientsAndDurationsVariables::
    SplineCoefficientsAndDurationsVariables(
        const trajectory::Spline& splineToClone)
  : SplineVariables(splineToClone)
{
  updateDimension();
}

//==============================================================================
std::shared_ptr<Variable> SplineCoefficientsAndDurationsVariables::clone() const
{
  return std::make_shared<SplineCoefficientsAndDurationsVariables>(*this);
}

//==============================================================================
void SplineCoefficientsAndDurationsVariables::setValue(
    const Eigen::VectorXd& value)
{
  // TODO(JS): Check the dimension of variables

  int index = 0;
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    const auto rows = mSpline.getStateSpace()->getDimension();
    const auto cols = mSpline.getSegmentCoefficients(i).cols();
    const auto numLocalVariables = rows * cols;

    Eigen::Map<const Eigen::MatrixXd> newSegmentCoeffis(
        value.segment(index, numLocalVariables).data(), rows, cols);
    mSpline.setSegmentCoefficients(i, newSegmentCoeffis);
    mSpline.setSegmentDuration(i, value[index + numLocalVariables]);

    index += numLocalVariables + 1;
  }

  assert(static_cast<std::size_t>(index) == getDimension());
}

//==============================================================================
Eigen::VectorXd SplineCoefficientsAndDurationsVariables::getValue() const
{
  Eigen::VectorXd value(getDimension());

  int index = 0;
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    auto& segmentCoeffs = mSpline.getSegmentCoefficients(i);

    const auto rows = mSpline.getStateSpace()->getDimension();
    const auto cols = segmentCoeffs.cols();
    const auto numLocalVariables = rows * cols;

    value.segment(index, numLocalVariables) = Eigen::Map<const Eigen::VectorXd>(
        segmentCoeffs.data(), numLocalVariables);
    value[index + numLocalVariables] = mSpline.getSegmentDuration(i);

    index += numLocalVariables + 1;
  }

  assert(static_cast<std::size_t>(index) == getDimension());
  return value;
}

//==============================================================================
void SplineCoefficientsAndDurationsVariables::setCoefficientValue(
    const Eigen::VectorXd& value)
{
  // TODO(JS): Check dimension of values (== num_segments * statespace_dim *
  // cols)

  int index = 0;
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    const auto rows = mSpline.getStateSpace()->getDimension();
    const auto cols = mSpline.getSegmentCoefficients(i).cols();
    const auto numLocalVariables = rows * cols;

    Eigen::Map<const Eigen::MatrixXd> newSegmentCoeffis(
        value.segment(index, numLocalVariables).data(), rows, cols);
    mSpline.setSegmentCoefficients(i, newSegmentCoeffis);

    index += numLocalVariables + 1;
  }

  assert(static_cast<std::size_t>(index) == getDimension());
}

//==============================================================================
void SplineCoefficientsAndDurationsVariables::setCoefficientValueTo(
    Eigen::VectorXd& vector, double value) const
{
  vector.resize(getDimension());

  int index = 0;
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    const auto rows = mSpline.getStateSpace()->getDimension();
    const auto cols = mSpline.getSegmentCoefficients(i).cols();
    const auto numLocalVariables = rows * cols;

    vector.segment(index, numLocalVariables).setConstant(value);

    index += numLocalVariables + 1;
  }

  assert(static_cast<std::size_t>(index) == getDimension());
}

//==============================================================================
void SplineCoefficientsAndDurationsVariables::setCoefficientValueTo(
    Eigen::VectorXd& vector, const Eigen::VectorXd& values) const
{
  vector.resize(getDimension());

  int index = 0;
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    const auto rows = mSpline.getStateSpace()->getDimension();
    const auto cols = mSpline.getSegmentCoefficients(i).cols();
    const auto numLocalVariables = rows * cols;

    assert(static_cast<std::size_t>(values.size()) == numLocalVariables);
    vector.segment(index, numLocalVariables) = values;

    index += numLocalVariables + 1;
  }

  assert(static_cast<std::size_t>(index) == getDimension());
}

//==============================================================================
void SplineCoefficientsAndDurationsVariables::setCoefficientValueTo(
    Eigen::VectorXd& vector, std::size_t index, double value) const
{
  // TODO(JS): Check validity of index (0 <= index < dimension_of_statespace)

  vector.resize(getDimension());

  int segmentIndex = 0;
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    const auto rows = mSpline.getStateSpace()->getDimension();
    const auto cols = mSpline.getSegmentCoefficients(i).cols();
    const auto numLocalVariables = rows * cols;

    vector[segmentIndex + index] = value;

    segmentIndex += numLocalVariables + 1;
  }

  assert(static_cast<std::size_t>(segmentIndex) == getDimension());
}

//==============================================================================
void SplineCoefficientsAndDurationsVariables::updateDimension()
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

//==============================================================================
void setCoefficientValueAsJointPositionLowerLimitsTo(
    Eigen::VectorXd& vector,
    const SplineCoefficientsAndDurationsVariables& variables,
    const dart::dynamics::MetaSkeleton& skeleton)
{
  variables.setCoefficientValueTo(vector, skeleton.getPositionLowerLimits());
}

//==============================================================================
void setCoefficientValueAsJointPositionUpperLimitsTo(
    Eigen::VectorXd& vector,
    const SplineCoefficientsAndDurationsVariables& variables,
    const dart::dynamics::MetaSkeleton& skeleton)
{
  variables.setCoefficientValueTo(vector, skeleton.getPositionUpperLimits());
}

} // namespace optimization
} // namespace planner
} // namespace aikido
