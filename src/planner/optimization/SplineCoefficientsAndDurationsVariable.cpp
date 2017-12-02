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

} // namespace optimization
} // namespace planner
} // namespace aikido
