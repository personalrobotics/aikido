#include <exception>
#include <string>
#include <boost/numeric/odeint.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpaceSaver.hpp>
#include <aikido/trajectory/Spline.hpp>

using aikido::statespace::dart::MetaSkeletonStateSpaceSaver;

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
static void checkDofLimits(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    Eigen::VectorXd const& q,
    Eigen::VectorXd const& qd)
{
  using dart::dynamics::DegreeOfFreedom;
  std::stringstream ss;

  for (std::size_t i = 0; i < stateSpace->getMetaSkeleton()->getNumDofs(); ++i)
  {
    const DegreeOfFreedom* const dof = stateSpace->getMetaSkeleton()->getDof(i);

    if (q[i] < dof->getPositionLowerLimit())
    {
      ss << "DOF " << dof->getName() << " exceeds lower position limit: ";
      ss << q[i] << " < " << dof->getPositionLowerLimit();
      throw DofLimitError(dof, ss.str());
    }
    else if (q[i] > dof->getPositionUpperLimit())
    {
      ss << "DOF " << dof->getName() << " exceeds upper position limit: ";
      ss << q[i] << " > " << dof->getPositionUpperLimit();
      throw DofLimitError(dof, ss.str());
    }
    else if (qd[i] < dof->getVelocityLowerLimit())
    {
      ss << "DOF " << dof->getName() << " exceeds lower velocity limit: ";
      ss << qd[i] << " < " << dof->getVelocityLowerLimit();
      throw DofLimitError(dof, ss.str());
    }
    else if (qd[i] > dof->getVelocityUpperLimit())
    {
      ss << "DOF " << dof->getName() << " exceeds upper velocity limit: ";
      ss << qd[i] << " > " << dof->getVelocityUpperLimit();
      throw DofLimitError(dof, ss.str());
    }
  }
}

//==============================================================================
static void checkCollision(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const aikido::constraint::TestablePtr& constraint)
{
  // Get current position
  auto state = stateSpace->getScopedStateFromMetaSkeleton();
  // Throw a termination if in collision
  if (!constraint->isSatisfied(state))
  {
    throw StateInCollisionError();
  }
}

//==============================================================================
VectorFieldPlanner::VectorFieldPlanner(
    const ConfigurationSpaceVectorFieldPtr vectorField,
    const aikido::constraint::TestablePtr constraint,
    double initialStepSize)
  : mVectorField(vectorField)
  , mConstraint(constraint)
  , mMetaSkeletonStateSpace(vectorField->getMetaSkeletonStateSpace())
  , mMetaSkeleton(vectorField->getMetaSkeleton())
  , mBodyNode(vectorField->getBodyNode())
  , mInitialStepSize(initialStepSize)
{
  mCacheIndex = -1;
  mIndex = 0;

  mEnableCollisionCheck = true;
  mEnableDofLimitCheck = true;
}

//==============================================================================
void VectorFieldPlanner::step(
    const Eigen::VectorXd& q, Eigen::VectorXd& qd, double /*_t*/)
{
  // set joint values
  mMetaSkeleton->setPositions(q);

  // collision checking of current joint values
  if (mEnableCollisionCheck)
  {
    checkCollision(mMetaSkeletonStateSpace, mConstraint);
  }

  // compute joint velocities
  bool success = mVectorField->getJointVelocities(qd);
  if (!success)
  {
    throw IntegrationFailedError();
  }

  if (mEnableDofLimitCheck)
  {
    checkDofLimits(mMetaSkeletonStateSpace, q, qd);
  }
}

//==============================================================================
void VectorFieldPlanner::check(const Eigen::VectorXd& q, double t)
{
  if (mTimer.getElapsedTime() > mTimelimit)
  {
    throw TimeLimitError();
  }

  // set joint values
  mMetaSkeleton->setPositions(q);

  // collision checking of current joint valuesb
  if (mEnableCollisionCheck)
  {
    checkCollision(mMetaSkeletonStateSpace, mConstraint);
  }

  Knot knot;
  knot.mT = t;
  knot.mPositions = q;

  mKnots.push_back(knot);
  mIndex += 1;

  VectorFieldPlannerStatus status = mVectorField->checkPlanningStatus();
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
std::unique_ptr<aikido::trajectory::Spline> VectorFieldPlanner::plan(
    double integrationTimeInterval,
    double timelimit,
    double useCollisionChecking,
    double useDofLimitChecking)
{
  using namespace std::placeholders;
  using errorStepper = boost::numeric::odeint::
      runge_kutta_dopri5<Eigen::VectorXd,
                         double,
                         Eigen::VectorXd,
                         double,
                         boost::numeric::odeint::vector_space_algebra>;

  auto saver = MetaSkeletonStateSpaceSaver(mMetaSkeletonStateSpace);
  DART_UNUSED(saver);

  mTimelimit = timelimit;
  mEnableCollisionCheck = useCollisionChecking;
  mEnableDofLimitCheck = useDofLimitChecking;

  if (mMetaSkeleton->getPositionUpperLimits()
      == mMetaSkeleton->getPositionLowerLimits())
  {
    throw std::invalid_argument("State space volume zero");
  }
  if (mMetaSkeleton->getVelocityUpperLimits()
      == mMetaSkeleton->getVelocityLowerLimits())
  {
    throw std::invalid_argument("Velocity space volume zero");
  }

  for (std::size_t i = 0; i < mMetaSkeleton->getNumDofs(); ++i)
  {
    std::stringstream ss;
    if (mMetaSkeleton->getPositionLowerLimit(i)
        > mMetaSkeleton->getPositionUpperLimit(i))
    {
      ss << "Position lower limit is larger than upper limit at DOF " << i;
      throw std::invalid_argument(ss.str());
    }
    if (mMetaSkeleton->getVelocityLowerLimit(i)
        > mMetaSkeleton->getVelocityUpperLimit(i))
    {
      ss << "Velocity lower limit is larger than upper limit at DOF " << i;
      throw std::invalid_argument(ss.str());
    }
  }

  mKnots.clear();
  mCacheIndex = -1;
  mIndex = 0;

  mTimer.start();
  try
  {
    Eigen::VectorXd initialQ = mMetaSkeleton->getPositions();
    // Integrate the vector field to get a configuration space path.
    boost::numeric::odeint::integrate_adaptive(
        errorStepper(),
        std::bind(&VectorFieldPlanner::step, this, _1, _2, _3),
        initialQ,
        0.,
        integrationTimeInterval,
        mInitialStepSize,
        std::bind(&VectorFieldPlanner::check, this, _1, _2));
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

  return convertToSpline(mKnots, mCacheIndex, mMetaSkeletonStateSpace);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const Eigen::Vector3d& direction,
    double distance,
    double maxDistance,
    double positionTolerance,
    double angularTolerance,
    double linearVelocityGain,
    bool useCollisionChecking,
    bool useDofLimitChecking,
    double initialStepSize,
    double jointLimitTolerance,
    double optimizationTolerance,
    double timelimit,
    double integralTimeInterval)
{
  if (distance < 0.)
  {
    std::stringstream ss;
    ss << "Distance must be non-negative; got " << distance << ".";
    throw std::runtime_error(ss.str());
  }

  if (maxDistance < distance)
  {
    throw std::runtime_error("Max distance is less than distance.");
  }

  if (direction.norm() == 0.0)
  {
    throw std::runtime_error("Direction vector is a zero vector");
  }

  auto vectorfield = std::make_shared<MoveEndEffectorOffsetVectorField>(
      stateSpace,
      bn,
      direction,
      distance,
      maxDistance,
      positionTolerance,
      angularTolerance,
      linearVelocityGain,
      initialStepSize,
      jointLimitTolerance,
      optimizationTolerance);

  auto planner = std::make_shared<VectorFieldPlanner>(vectorfield, constraint);
  return planner->plan(
      integralTimeInterval,
      timelimit,
      useCollisionChecking,
      useDofLimitChecking);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorPose(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const Eigen::Isometry3d& goalPose,
    double poseErrorTolerance,
    double linearVelocityGain,
    double angularvelocityGain,
    bool useCollisionChecking,
    bool useDofLimitChecking,
    double initialStepSize,
    double jointLimitTolerance,
    double optimizationTolerance,
    double timelimit,
    double integralTimeInterval)
{
  auto vectorfield = std::make_shared<MoveEndEffectorPoseVectorField>(
      stateSpace,
      bn,
      goalPose,
      poseErrorTolerance,
      linearVelocityGain,
      angularvelocityGain,
      initialStepSize,
      jointLimitTolerance,
      optimizationTolerance);

  auto planner = std::make_shared<VectorFieldPlanner>(vectorfield, constraint);
  return planner->plan(
      integralTimeInterval,
      timelimit,
      useCollisionChecking,
      useDofLimitChecking);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> planWorkspacePath(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const aikido::trajectory::InterpolatedPtr workspacePath,
    double positionTolerance,
    double angularTolerance,
    double tStep,
    const Eigen::Vector6d& kpFF,
    const Eigen::Vector6d& kpE,
    double useCollisionChecking,
    double useDofLimitChecking,
    double initialStepSize,
    double jointLimitTolerance,
    double optimizationTolerance,
    double timelimit,
    double integralTimeInterval)
{
  auto vectorfield
      = std::make_shared<MoveEndEffectorAlongWorkspacePathVectorField>(
          stateSpace,
          bn,
          workspacePath,
          positionTolerance,
          angularTolerance,
          tStep,
          initialStepSize,
          jointLimitTolerance,
          optimizationTolerance,
          kpFF,
          kpE);

  auto planner = std::make_shared<VectorFieldPlanner>(vectorfield, constraint);
  return planner->plan(
      integralTimeInterval,
      timelimit,
      useCollisionChecking,
      useDofLimitChecking);
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
