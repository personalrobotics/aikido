#include "aikido/robot/Robot.hpp"

#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/planner/kunzretimer/KunzRetimer.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace robot {

using constraint::ConstTestablePtr;
using constraint::TestablePtr;
using constraint::dart::CollisionFreePtr;
using constraint::dart::TSRPtr;
using planner::TrajectoryPostProcessor;
using planner::kunzretimer::KunzRetimer;
using planner::parabolic::ParabolicSmoother;
using planner::parabolic::ParabolicTimer;
using statespace::StateSpace;
using statespace::StateSpacePtr;
using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSpace;
using statespace::dart::MetaSkeletonStateSpacePtr;
using trajectory::Interpolated;
using trajectory::InterpolatedPtr;
using trajectory::Spline;
using trajectory::TrajectoryPtr;
using trajectory::UniqueSplinePtr;

using dart::dynamics::BodyNodePtr;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;

// TODO: Temporary constants for planning calls.
// These should be defined when we construct planner adapter classes
// static const double collisionResolution = 0.1;
static constexpr double kAsymmetryTolerance = 1e-3;

namespace {

// TODO: These may not generalize to many robots.
Eigen::VectorXd getSymmetricLimits(
    const MetaSkeleton& metaSkeleton,
    const Eigen::VectorXd& lowerLimits,
    const Eigen::VectorXd& upperLimits,
    const std::string& limitName,
    double kAsymmetryTolerance)
{
  const auto numDofs = metaSkeleton.getNumDofs();
  assert(static_cast<std::size_t>(lowerLimits.size()) == numDofs);
  assert(static_cast<std::size_t>(upperLimits.size()) == numDofs);

  Eigen::VectorXd symmetricLimits(numDofs);
  for (std::size_t iDof = 0; iDof < numDofs; ++iDof)
  {
    symmetricLimits[iDof] = std::min(-lowerLimits[iDof], upperLimits[iDof]);
    if (std::abs(lowerLimits[iDof] + upperLimits[iDof]) > kAsymmetryTolerance)
    {
      dtwarn << "MetaSkeleton '" << metaSkeleton.getName()
             << "' has asymmetric " << limitName << " limits ["
             << lowerLimits[iDof] << ", " << upperLimits[iDof]
             << "] for DegreeOfFreedom '"
             << metaSkeleton.getDof(iDof)->getName() << "' (index: " << iDof
             << "). Using a conservative limit of" << symmetricLimits[iDof]
             << ".";
    }
  }
  return symmetricLimits;
}

Eigen::VectorXd getSymmetricVelocityLimits(
    const MetaSkeleton& metaSkeleton, double kAsymmetryTolerance)
{
  return getSymmetricLimits(
      metaSkeleton,
      metaSkeleton.getVelocityLowerLimits(),
      metaSkeleton.getVelocityUpperLimits(),
      "velocity",
      kAsymmetryTolerance);
}

Eigen::VectorXd getSymmetricAccelerationLimits(
    const MetaSkeleton& metaSkeleton, double kAsymmetryTolerance)
{
  return getSymmetricLimits(
      metaSkeleton,
      metaSkeleton.getAccelerationLowerLimits(),
      metaSkeleton.getAccelerationUpperLimits(),
      "acceleration",
      kAsymmetryTolerance);
}

} // namespace

//==============================================================================
Robot::Robot(
    const std::string& name,
    MetaSkeletonPtr metaSkeleton,
    common::UniqueRNGPtr rng,
    control::TrajectoryExecutorPtr trajectoryExecutor,
    dart::collision::CollisionDetectorPtr collisionDetector,
    std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
        selfCollisionFilter)
  : mRootRobot(this)
  , mName(name)
  , mMetaSkeleton(metaSkeleton)
  , mStateSpace(std::make_shared<MetaSkeletonStateSpace>(mMetaSkeleton.get()))
  , mParentSkeleton(nullptr)
  , mRng(std::move(rng))
  , mTrajectoryExecutor(std::move(trajectoryExecutor))
  , mCollisionDetector(collisionDetector)
  , mSelfCollisionFilter(selfCollisionFilter)
{
  if (!mMetaSkeleton)
  {
    throw std::invalid_argument("Robot is nullptr.");
  }
  mParentSkeleton = mMetaSkeleton->getBodyNode(0)->getSkeleton();
}

//==============================================================================
std::string Robot::getName() const
{
  return mName;
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr Robot::getMetaSkeleton() const
{
  return mMetaSkeleton;
}

//==============================================================================
dart::dynamics::MetaSkeletonPtr Robot::getMetaSkeleton()
{
  return std::const_pointer_cast<dart::dynamics::MetaSkeleton>(
      const_cast<const Robot*>(this)->getMetaSkeleton());
}

//==============================================================================
statespace::dart::ConstMetaSkeletonStateSpacePtr Robot::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
statespace::dart::MetaSkeletonStateSpacePtr Robot::getStateSpace()
{
  return std::const_pointer_cast<statespace::dart::MetaSkeletonStateSpace>(
      const_cast<const Robot*>(this)->getStateSpace());
}

//==============================================================================
boost::optional<Eigen::VectorXd> Robot::getNamedConfiguration(
    const std::string& name) const
{
  auto configuration = mNamedConfigurations.find(name);
  if (configuration == mNamedConfigurations.end())
  {
    return boost::none;
  }

  return configuration->second;
}

//==============================================================================
void Robot::setNamedConfigurations(
    std::unordered_map<std::string, const Eigen::VectorXd> namedConfigurations)
{
  mNamedConfigurations = std::move(namedConfigurations);
}

//=============================================================================
void Robot::setRoot(Robot* robot)
{
  if (robot == nullptr)
  {
    throw std::invalid_argument("ConcreteRobot is null.");
  }

  mRootRobot = robot;
}

//==============================================================================
std::future<void> Robot::executeTrajectory(
    const TrajectoryPtr& trajectory) const
{
  return mTrajectoryExecutor->execute(trajectory);
}

//==============================================================================
void Robot::step(const std::chrono::system_clock::time_point& timepoint)
{
  // Assumes that the parent robot is locked
  mTrajectoryExecutor->step(timepoint);
}

// ==============================================================================
CollisionFreePtr Robot::getSelfCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton) const
{
  using constraint::dart::CollisionFree;

  if (mRootRobot != this)
  {
    return mRootRobot->getSelfCollisionConstraint(space, metaSkeleton);
  }

  mParentSkeleton->enableSelfCollisionCheck();
  mParentSkeleton->disableAdjacentBodyCheck();

  // TODO: Switch to PRIMITIVE once this is fixed in DART.
  // mCollisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  auto collisionOption
      = dart::collision::CollisionOption(false, 1, mSelfCollisionFilter);
  auto collisionFreeConstraint = std::make_shared<CollisionFree>(
      space, metaSkeleton, mCollisionDetector, collisionOption);
  collisionFreeConstraint->addSelfCheck(
      mCollisionDetector->createCollisionGroupAsSharedPtr(mMetaSkeleton.get()));
  return collisionFreeConstraint;
}

//=============================================================================
TestablePtr Robot::getFullCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const CollisionFreePtr& collisionFree) const
{
  if (mRootRobot != this)
  {
    return mRootRobot->getFullCollisionConstraint(
        space, metaSkeleton, collisionFree);
  }

  auto selfCollisionFree = getSelfCollisionConstraint(space, metaSkeleton);

  if (!collisionFree)
  {
    return selfCollisionFree;
  }

  // Make testable constraints for collision check
  std::vector<ConstTestablePtr> constraints;
  constraints.reserve(2);
  constraints.emplace_back(selfCollisionFree);
  if (collisionFree)
  {
    if (collisionFree->getStateSpace() != space)
    {
      throw std::runtime_error("CollisionFree has incorrect statespace.");
    }
    constraints.emplace_back(collisionFree);
  }

  return std::make_shared<constraint::TestableIntersection>(space, constraints);
}

//==============================================================================
Eigen::VectorXd Robot::getVelocityLimits(const MetaSkeleton& metaSkeleton) const
{
  return getSymmetricVelocityLimits(metaSkeleton, kAsymmetryTolerance);
}

//==============================================================================
Eigen::VectorXd Robot::getAccelerationLimits(
    const MetaSkeleton& metaSkeleton) const
{
  return getSymmetricAccelerationLimits(metaSkeleton, kAsymmetryTolerance);
}

//==============================================================================
std::unique_ptr<common::RNG> Robot::cloneRNG()
{
  return mRng->clone();
}

} // namespace robot
} // namespace aikido
