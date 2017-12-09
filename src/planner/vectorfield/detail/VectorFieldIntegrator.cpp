#include "VectorFieldIntegrator.hpp"
#include <exception>
#include <string>
#include <aikido/common/Spline.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/trajectory/Spline.hpp>
#include "VectorFieldPlannerExceptions.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {
namespace detail {

constexpr double defaultConstraintCheckResolution = 1e-3;

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const std::vector<Knot>& knots,
    aikido::statespace::ConstStateSpacePtr stateSpace)
{
  using dart::common::make_unique;

  std::size_t dimension = stateSpace->getDimension();

  // TODO: const cast will be removed after the spline constructor updated.
  auto nonConstStateSpace
      = std::const_pointer_cast<aikido::statespace::StateSpace>(stateSpace);
  auto outputTrajectory
      = make_unique<aikido::trajectory::Spline>(nonConstStateSpace);

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
VectorFieldIntegrator::VectorFieldIntegrator(
    const VectorField* vectorField,
    const aikido::constraint::Testable* collisionFreeConstraint,
    double timelimit,
    double checkConstraintResolution)
  : mVectorField(vectorField)
  , mConstraint(collisionFreeConstraint)
  , mTimelimit(timelimit)
  , mConstraintCheckResolution(checkConstraintResolution)
{
  mCacheIndex = -1;
  mIndex = 0;

  mDimension = mVectorField->getStateSpace()->getDimension();
}

//==============================================================================
void VectorFieldIntegrator::start()
{
  mTimer.start();
  mKnots.clear();
  mCacheIndex = -1;
  mIndex = 0;
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
void VectorFieldIntegrator::step(
    const Eigen::VectorXd& q, Eigen::VectorXd& qd, double /*_t*/)
{
  aikido::statespace::StateSpace::State* state
      = mVectorField->getStateSpace()->allocateState();
  mVectorField->getStateSpace()->expMap(q, state);

  // compute joint velocities
  bool success = mVectorField->evaluateVelocity(state, qd);
  mVectorField->getStateSpace()->freeState(state);
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
    // throw TimeLimitError();
  }

  aikido::statespace::StateSpace::State* state
      = mVectorField->getStateSpace()->allocateState();
  mVectorField->getStateSpace()->expMap(q, state);

  Knot knot;
  knot.mT = t;
  knot.mPositions = q;

  double trajEvalStartTime = 0;
  if (mKnots.size() > 0)
  {
    // create new incremental segment
    std::vector<Knot> segment;
    segment.push_back(mKnots.back());
    segment.push_back(knot);
    auto trajSegment = convertToSpline(segment, mVectorField->getStateSpace());

    if (mKnots.size() > 1)
    {
      trajEvalStartTime = mConstraintCheckResolution;
    }
    if (!mVectorField->evaluateTrajectory(
            *trajSegment.get(),
            mConstraint,
            mConstraintCheckResolution,
            trajEvalStartTime))
    {
      throw VectorFieldTerminated("State not in constraint.");
    }
  }
  mKnots.push_back(knot);
  mIndex += 1;

  VectorFieldPlannerStatus status = mVectorField->evaluateStatus(state);

  mVectorField->getStateSpace()->freeState(state);

  if (status == VectorFieldPlannerStatus::CACHE_AND_CONTINUE
      || status == VectorFieldPlannerStatus::CACHE_AND_TERMINATE)
  {
    mCacheIndex = mIndex;
  }

  if (status == VectorFieldPlannerStatus::TERMINATE
      || status == VectorFieldPlannerStatus::CACHE_AND_TERMINATE)
  {
    throw VectorFieldTerminated(
        "Planning was terminated by the StatusCallback.");
  }
}

} // namesapce detail
} // namespace vectorfield
} // namespace planner
} // namespace aikido
