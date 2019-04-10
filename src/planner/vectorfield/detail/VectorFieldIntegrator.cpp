#include "VectorFieldIntegrator.hpp"
#include <exception>
#include <string>
#include <aikido/common/Spline.hpp>
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
      = ::dart::common::make_unique<aikido::trajectory::Spline>(stateSpace);

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
      = ::dart::common::make_unique<aikido::trajectory::Interpolated>(
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
  // std::cout << __LINE__ << " " << __FILE__ << std::endl;
  mVectorField->getStateSpace()->expMap(q, mState);

  // compute joint velocities
  // std::cout << __LINE__ << " " << __FILE__ << std::endl;
  bool success = mVectorField->evaluateVelocity(mState, qd);
  if (!success)
  {
    // std::cout << __LINE__ << " " << __FILE__ << std::endl;
    throw IntegrationFailedError();
  }
  // std::cout << __LINE__ << " " << __FILE__ << std::endl;
}

//==============================================================================
void VectorFieldIntegrator::check(const Eigen::VectorXd& q, double t)
{
  // std::cout << __LINE__ << " " << __FILE__ << std::endl;
  if (mTimer.getElapsedTime() > mTimelimit)
  {
    std::cout << __LINE__ << " " << __FILE__ << std::endl;
    throw TimeLimitError();
  }

// std::cout << __LINE__ << " " << __FILE__ << std::endl;
  mVectorField->getStateSpace()->expMap(q, mState);

// std::cout << __LINE__ << " " << __FILE__ << std::endl;
  Knot knot;
  knot.mT = t;
  knot.mPositions = q;
  mKnots.push_back(knot);

// std::cout << __LINE__ << " " << __FILE__ << std::endl;
  if (mKnots.size() > 1)
  {
    // std::cout << __LINE__ << " " << __FILE__ << std::endl;
    auto trajSegment = convertToSpline(mKnots, mVectorField->getStateSpace());

// std::cout << __LINE__ << " " << __FILE__ << std::endl;
    if (!mVectorField->evaluateTrajectory(
            *trajSegment,
            mConstraint,
            mConstraintCheckResolution,
            mLastEvaluationTime,
            false))
    {
      // std::cout << __LINE__ << " " << __FILE__ << std::endl;
      throw ConstraintViolatedError();
    }
  }

// std::cout << __LINE__ << " " << __FILE__ << std::endl;
  VectorFieldPlannerStatus status = mVectorField->evaluateStatus(mState);

  if (status == VectorFieldPlannerStatus::CACHE_AND_CONTINUE
      || status == VectorFieldPlannerStatus::CACHE_AND_TERMINATE)
  {
    // std::cout << __LINE__ << " " << __FILE__ << std::endl;
    // std::cout << mKnots.size() << std::endl;
    mCacheIndex = mKnots.size();
  }

  if (status == VectorFieldPlannerStatus::TERMINATE
      || status == VectorFieldPlannerStatus::CACHE_AND_TERMINATE)
  {
    // std::cout << __LINE__ << " " << __FILE__ << std::endl;
    throw VectorFieldTerminated(
        "Planning was terminated by the StatusCallback.");
  }
}

} // namespace detail
} // namespace vectorfield
} // namespace planner
} // namespace aikido
