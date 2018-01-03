#include "aikido/planner/optimization/SplineCoefficientsVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
SplineCoefficientsVariables::SplineCoefficientsVariables(
    const trajectory::Spline& splineToClone,
    bool fixedStartPoint,
    bool fixedEndPoint)
  : SplineVariable(splineToClone)
  , mIsFixedStartPoint(fixedStartPoint)
  , mIsFixedEndPoint(fixedEndPoint)
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<Variable> SplineCoefficientsVariables::clone() const
{
  return dart::common::make_unique<SplineCoefficientsVariables>(*this);
}

//==============================================================================
void SplineCoefficientsVariables::setValue(const Eigen::VectorXd& variables)
{
  // TODO(JS): Check the dimension of variables

  const auto& statespace = mSpline.getStateSpace();

  int index = 0;
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    const auto rows = statespace->getDimension();
    const auto cols = mSpline.getSegmentCoefficients(i).cols();
    const auto numLocalVariables = rows * cols;

    Eigen::Map<const Eigen::MatrixXd> newSegmentCoeffis(
        variables.segment(index, numLocalVariables).data(), rows, cols);
    mSpline.setSegmentCoefficients(i, newSegmentCoeffis);

    index += numLocalVariables;
  }

  assert(static_cast<std::size_t>(index) == getDimension());
}

//==============================================================================
Eigen::VectorXd SplineCoefficientsVariables::getValue() const
{
  Eigen::VectorXd variable(getDimension());

  const auto& statespace = mSpline.getStateSpace();

  int index = 0;
  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    auto& segmentCoeffs = mSpline.getSegmentCoefficients(i);

    const auto rows = statespace->getDimension();
    const auto cols = segmentCoeffs.cols();
    const auto numLocalVariables = rows * cols;

    variable.segment(index, numLocalVariables)
        = Eigen::Map<const Eigen::VectorXd>(
            segmentCoeffs.data(), numLocalVariables);

    index += numLocalVariables;
  }

  assert(static_cast<std::size_t>(index) == getDimension());

  return variable;
}

//==============================================================================
void SplineCoefficientsVariables::freeStartPoint()
{
  if (!mIsFixedStartPoint)
    return;

  mIsFixedStartPoint = false;
  mNeedDimensionUpdate = true;
}

//==============================================================================
void SplineCoefficientsVariables::fixStartPoint()
{
  if (mIsFixedStartPoint)
    return;

  mIsFixedStartPoint = true;
  mNeedDimensionUpdate = true;
}

//==============================================================================
bool SplineCoefficientsVariables::isStartPointFixed() const
{
  return mIsFixedStartPoint;
}

//==============================================================================
void SplineCoefficientsVariables::freeEndPoint()
{
  if (!mIsFixedEndPoint)
    return;

  mIsFixedEndPoint = false;
  mNeedDimensionUpdate = true;
}

//==============================================================================
void SplineCoefficientsVariables::fixEndPoint()
{
  if (mIsFixedEndPoint)
    return;

  mIsFixedEndPoint = true;
  mNeedDimensionUpdate = true;
}

//==============================================================================
bool SplineCoefficientsVariables::isEndPointFixed() const
{
  return mIsFixedEndPoint;
}

//==============================================================================
void SplineCoefficientsVariables::updateDimension() const
{
  std::size_t dim = 0u;
  const auto& statespace = mSpline.getStateSpace();

  for (auto i = 0u; i < mSpline.getNumSegments(); ++i)
  {
    const auto& segmentCoeffs = mSpline.getSegmentCoefficients(i);

    // This should be guaranteed by Spline::addSegment(). Double check this for
    // sure.
    assert(
        static_cast<std::size_t>(segmentCoeffs.rows())
        == statespace->getDimension());

    dim += segmentCoeffs.cols();
  }

  dim *= statespace->getDimension();

  mDimension = dim;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
