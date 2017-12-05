#include <exception>
#include <string>
#include <boost/numeric/odeint.hpp>
#include <aikido/planner/vectorfield/VectorFieldIntegrator.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

constexpr double integrationTimeInterval = 10.0;
constexpr double defaultConstraintCheckResolution = 1e-3;

//==============================================================================
VectorFieldIntegrator::VectorFieldIntegrator(
    const VectorFieldPtr vectorField,
    const aikido::constraint::TestablePtr collisionFreeConstraint,
    double initialStepSize)
  : mVectorField(vectorField)
  , mCollisionFreeConstraint(collisionFreeConstraint)
  , mInitialStepSize(initialStepSize)
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
  if (!convertPositionsToState(q, state))
  {
    throw IntegrationFailedError();
  }

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
  convertPositionsToState(q, state);

  Knot knot;
  knot.mT = t;
  knot.mPositions = q;

  if (mKnots.size() > 0)
  {
    // create a temporary trajectory segment
    std::vector<Knot> segment;
    segment.push_back(mKnots.back());
    segment.push_back(knot);
    auto trajSegment = convertToSpline(segment, mVectorField->getStateSpace());

    evaluateTrajectory(
        trajSegment.get(),
        mCollisionFreeConstraint,
        mConstraintCheckResolution);
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
    const aikido::statespace::StateSpace::State* startState, double timelimit)
{
  using namespace std::placeholders;
  using errorStepper = boost::numeric::odeint::
      runge_kutta_dopri5<Eigen::VectorXd,
                         double,
                         Eigen::VectorXd,
                         double,
                         boost::numeric::odeint::vector_space_algebra>;

  mTimelimit = timelimit;
  mTimer.start();

  mKnots.clear();
  mCacheIndex = -1;
  mIndex = 0;

  try
  {
    Eigen::VectorXd initialQ(mDimension);
    convertStateToPositions(startState, initialQ);
    // Integrate the vector field to get a configuration space path.
    boost::numeric::odeint::integrate_adaptive(
        errorStepper(),
        std::bind(&VectorFieldIntegrator::step, this, _1, _2, _3),
        initialQ,
        0.,
        integrationTimeInterval,
        mInitialStepSize,
        std::bind(&VectorFieldIntegrator::check, this, _1, _2));
  }
  catch (const VectorFieldTerminated& e)
  {
    dtwarn << e.what() << std::endl;
  }
  catch (const IntegrationFailedError& e)
  {
    dtwarn << e.what() << std::endl;
    return nullptr;
  }

  if (mCacheIndex < 0)
  {
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

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline>
VectorFieldIntegrator::convertToSpline(
    const std::vector<Knot>& knots,
    const aikido::statespace::StateSpacePtr stateSpace)
{
  using dart::common::make_unique;

  auto outputTrajectory = make_unique<aikido::trajectory::Spline>(stateSpace);

  using CubicSplineProblem = aikido::common::
      SplineProblem<double, int, 2, Eigen::Dynamic, Eigen::Dynamic>;

  const Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(mDimension);
  auto currState = stateSpace->createState();
  for (std::size_t iknot = 0; iknot < knots.size() - 1; ++iknot)
  {
    const double segmentDuration = knots[iknot + 1].mT - knots[iknot].mT;
    Eigen::VectorXd currentPosition = knots[iknot].mPositions;
    Eigen::VectorXd nextPosition = knots[iknot + 1].mPositions;

    if (segmentDuration == 0.0)
    {
      std::cout << "ZERO SEGMENT DURATION " << std::endl;
    }
    CubicSplineProblem problem(
        Eigen::Vector2d{0., segmentDuration}, 2, mDimension);
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
double VectorFieldIntegrator::getInitialStepSize()
{
  return mInitialStepSize;
}

//==============================================================================
double VectorFieldIntegrator::getConstraintCheckResolution()
{
  return mConstraintCheckResolution;
}

//==============================================================================
void VectorFieldIntegrator::setConstraintCheckResolution(double resolution)
{
  mConstraintCheckResolution = resolution;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
