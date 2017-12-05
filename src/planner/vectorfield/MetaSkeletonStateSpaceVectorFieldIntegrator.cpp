#include <exception>
#include <string>
#include <boost/numeric/odeint.hpp>
#include <aikido/planner/vectorfield/MetaSkeletonStateSpaceVectorFieldIntegrator.hpp>
#include <aikido/planner/vectorfield/detail/VectorFieldPlannerExceptions.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
MetaSkeletonStateSpaceVectorFieldIntegrator::
    MetaSkeletonStateSpaceVectorFieldIntegrator(
        const MetaSkeletonStateSpaceVectorFieldPtr vectorField,
        const aikido::constraint::TestablePtr constraint,
        double initialStepSize)
  : VectorFieldIntegrator(vectorField, constraint, initialStepSize)
  , mMetaSkeletonStateSpace(vectorField->getMetaSkeletonStateSpace())
  , mMetaSkeleton(vectorField->getMetaSkeleton())
{
  // Do nothing
}

//==============================================================================
bool MetaSkeletonStateSpaceVectorFieldIntegrator::evaluateTrajectory(
    const aikido::trajectory::Trajectory* trajectory,
    const aikido::constraint::TestablePtr collisionFreeConstraint,
    double evalStepSize)
{
  auto boundConstraint = mVectorField->getBoundTestable();
  for (double t = trajectory->getStartTime(); t <= trajectory->getEndTime();
       t += evalStepSize)
  {
    auto state = mMetaSkeletonStateSpace->createState();
    trajectory->evaluate(t, state);

    // firstly check the bound
    if (!boundConstraint->isSatisfied(state))
    {
      throw DofLimitError();
    }

    // last check collision free constraint
    if (!collisionFreeConstraint->isSatisfied(state))
    {
      throw StateInCollisionError();
    }
  }

  return true;
}

//==============================================================================
bool MetaSkeletonStateSpaceVectorFieldIntegrator::convertStateToPositions(
    const aikido::statespace::StateSpace::State* state,
    Eigen::VectorXd& positions)
{
  auto newState = static_cast<const aikido::statespace::dart::
                                  MetaSkeletonStateSpace::State*>(state);
  if (newState)
  {
    mMetaSkeletonStateSpace->convertStateToPositions(newState, positions);
    return true;
  }
  return false;
}

//==============================================================================
bool MetaSkeletonStateSpaceVectorFieldIntegrator::convertPositionsToState(
    const Eigen::VectorXd& positions,
    aikido::statespace::StateSpace::State* state)
{
  auto newState
      = static_cast<aikido::statespace::dart::MetaSkeletonStateSpace::State*>(
          state);
  if (newState)
  {
    mMetaSkeletonStateSpace->convertPositionsToState(positions, newState);
    return true;
  }
  return false;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
