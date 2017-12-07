#include <exception>
#include <string>
#include <boost/numeric/odeint.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/planner/vectorfield/VectorFieldIntegrator.hpp>
#include <aikido/planner/vectorfield/detail/VectorFieldPlannerExceptions.hpp>
#include <aikido/planner/vectorfield/detail/VectorFieldUtil.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

constexpr double integrationTimeInterval = 10.0;
constexpr double defaultConstraintCheckResolution = 1e-3;

//==============================================================================
VectorFieldIntegrator::VectorFieldIntegrator(
    const VectorFieldPtr vectorField,
    const aikido::constraint::TestablePtr collisionFreeConstraint)
  : mVectorField(vectorField), mConstraint(collisionFreeConstraint)
{
  mCacheIndex = -1;
  mIndex = 0;

  mDimension = mVectorField->getStateSpace()->getDimension();
  mConstraintCheckResolution = defaultConstraintCheckResolution;
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
    throw TimeLimitError();
  }

  aikido::statespace::StateSpace::State* state
      = mVectorField->getStateSpace()->allocateState();
  mVectorField->getStateSpace()->expMap(q, state);

  Knot knot;
  knot.mT = t;
  knot.mPositions = q;

  if (mKnots.size() > 0)
  {
    // create new incremental segment
    std::vector<Knot> segment;
    segment.push_back(mKnots.back());
    segment.push_back(knot);
    auto trajSegment = convertToSpline(segment, mVectorField->getStateSpace());

    mVectorField->evaluateTrajectory(
        *trajSegment.get(), mConstraint, mConstraintCheckResolution);
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

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline>
VectorFieldIntegrator::followVectorField(
    const aikido::statespace::StateSpace::State* startState,
    std::chrono::duration<double> timelimit,
    double initialStepSize,
    double checkConstraintResolution,
    planner::PlanningResult* planningResult)
{
  using namespace std::placeholders;
  using errorStepper = boost::numeric::odeint::
      runge_kutta_dopri5<Eigen::VectorXd,
                         double,
                         Eigen::VectorXd,
                         double,
                         boost::numeric::odeint::vector_space_algebra>;

  mTimelimit = timelimit.count();
  mConstraintCheckResolution = checkConstraintResolution;
  mTimer.start();

  mKnots.clear();
  mCacheIndex = -1;
  mIndex = 0;

  try
  {
    Eigen::VectorXd initialQ(mDimension);
    mVectorField->getStateSpace()->logMap(startState, initialQ);

    // Integrate the vector field to get a configuration space path.
    boost::numeric::odeint::integrate_adaptive(
        errorStepper(),
        std::bind(&VectorFieldIntegrator::step, this, _1, _2, _3),
        initialQ,
        0.,
        integrationTimeInterval,
        initialStepSize,
        std::bind(&VectorFieldIntegrator::check, this, _1, _2));
  }
  catch (const VectorFieldTerminated& e)
  {
    dtwarn << e.what() << std::endl;
    if (planningResult)
    {
      planningResult->message = e.what();
    }
  }
  catch (const VectorFieldError& e)
  {
    dtwarn << e.what() << std::endl;
    if (planningResult)
    {
      planningResult->message = e.what();
    }
    return nullptr;
  }

  if (mCacheIndex < 0)
  {
    if (planningResult)
    {
      planningResult->message = "No waypoint cached.";
    }
    return nullptr;
  }

  // Construct the output spline.
  Eigen::VectorXd times(mCacheIndex);
  std::transform(
      mKnots.begin(),
      mKnots.begin() + mCacheIndex,
      times.data(),
      [](const Knot& knot) { return knot.mT; });

  return convertToSpline(mKnots, mVectorField->getStateSpace());
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
