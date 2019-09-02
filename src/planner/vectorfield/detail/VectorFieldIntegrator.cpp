#include "VectorFieldIntegrator.hpp"
#include <exception>
#include <string>
#include <aikido/common/Spline.hpp>
#include <aikido/common/memory.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include "VectorFieldPlannerExceptions.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {
namespace detail {

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const std::vector<Knot>& knots,
    aikido::statespace::ConstStateSpacePtr stateSpace)
{
  std::size_t dimension = stateSpace->getDimension();

  auto outputTrajectory
      = ::aikido::common::make_unique<aikido::trajectory::Spline>(stateSpace);

  using CubicSplineProblem = aikido::common::
      SplineProblem<double, int, 2, Eigen::Dynamic, Eigen::Dynamic>;

  const Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(dimension);
  auto currState = stateSpace->createState();
  for (std::size_t iknot = 0; iknot < knots.size() - 1; ++iknot)
  {
    const double segmentDuration = knots[iknot + 1].mT - knots[iknot].mT;
    Eigen::VectorXd currentPosition = knots[iknot].mPositions;
    Eigen::VectorXd nextPosition = knots[iknot + 1].mPositions;

    CubicSplineProblem problem(
        Eigen::Vector2d{0., segmentDuration}, 2, dimension);
    problem.addConstantConstraint(0, 0, zeroPosition);
    problem.addConstantConstraint(1, 0, nextPosition - currentPosition);
    const auto solution = problem.fit();
    const auto coefficients = solution.getCoefficients().front();

    stateSpace->expMap(currentPosition, currState);
    outputTrajectory->addSegment(coefficients, segmentDuration, currState);
  }
  return outputTrajectory;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Interpolated> convertToInterpolated(
    const std::vector<Knot>& knots,
    aikido::statespace::ConstStateSpacePtr stateSpace)
{
  auto interpolator
      = std::make_shared<const aikido::statespace::GeodesicInterpolator>(
          stateSpace);
  auto outputTrajectory
      = ::aikido::common::make_unique<aikido::trajectory::Interpolated>(
          stateSpace, interpolator);

  auto currState = stateSpace->createState();
  for (const auto& knot : knots)
  {
    stateSpace->expMap(knot.mPositions, currState);
    outputTrajectory->addWaypoint(knot.mT, currState);
  }

  return outputTrajectory;
}

//==============================================================================
VectorFieldIntegrator::VectorFieldIntegrator(
    const VectorField* vectorField,
    const aikido::constraint::Testable* collisionFreeConstraint,
    double timelimit,
    double checkConstraintResolution)
  : mVectorField(vectorField)
  , mConstraint(collisionFreeConstraint)
  , mCacheIndex(-1)
  , mDimension(mVectorField->getStateSpace()->getDimension())
  , mTimelimit(timelimit)
  , mConstraintCheckResolution(checkConstraintResolution)
  , mState(mVectorField->getStateSpace()->createState())
  , mLastEvaluationTime(0.0)
{
  // Do nothing
}

//==============================================================================
void VectorFieldIntegrator::start()
{
  mTimer.start();
  mKnots.clear();
  mCacheIndex = -1;
  mLastEvaluationTime = 0.0;
}

//==============================================================================
int VectorFieldIntegrator::getCacheIndex()
{
  return mCacheIndex;
}

//==============================================================================
std::vector<Knot>& VectorFieldIntegrator::getKnots()
{
  return mKnots;
}

//==============================================================================
double VectorFieldIntegrator::getLastEvaluationTime()
{
  return mLastEvaluationTime;
}

//==============================================================================
void VectorFieldIntegrator::step(
    const Eigen::VectorXd& q, Eigen::VectorXd& qd, double /*_t*/)
{
  mVectorField->getStateSpace()->expMap(q, mState);

  // compute joint velocities
  bool success = mVectorField->evaluateVelocity(mState, qd);
  if (!success)
  {
    throw IntegrationFailedError();
  }
}

//==============================================================================
void VectorFieldIntegrator::check(const Eigen::VectorXd& q, double t)
{
  if (mTimer.getElapsedTime() > mTimelimit)
  {
    throw TimeLimitError();
  }

  mVectorField->getStateSpace()->expMap(q, mState);

  Knot knot;
  knot.mT = t;
  knot.mPositions = q;
  mKnots.push_back(knot);

  if (mKnots.size() > 1)
  {
    auto trajSegment = convertToSpline(mKnots, mVectorField->getStateSpace());

    if (!mVectorField->evaluateTrajectory(
            *trajSegment,
            mConstraint,
            mConstraintCheckResolution,
            mLastEvaluationTime,
            false))
    {
      throw ConstraintViolatedError();
    }
  }

  VectorFieldPlannerStatus status = mVectorField->evaluateStatus(mState);

  if (status == VectorFieldPlannerStatus::CACHE_AND_CONTINUE
      || status == VectorFieldPlannerStatus::CACHE_AND_TERMINATE)
  {
    mCacheIndex = mKnots.size();
  }

  if (status == VectorFieldPlannerStatus::TERMINATE
      || status == VectorFieldPlannerStatus::CACHE_AND_TERMINATE)
  {
    throw VectorFieldTerminated(
        "Planning was terminated by the StatusCallback.");
  }
}

} // namespace detail
} // namespace vectorfield
} // namespace planner
} // namespace aikido
