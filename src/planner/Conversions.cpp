#include <aikido/planner/Conversions.hpp>

#include <sstream>
#include <aikido/statespace/dart/RnJoint.hpp>
#include <aikido/statespace/dart/SO2Joint.hpp>

using aikido::statespace::CartesianProduct;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R1;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::statespace::dart::R1Joint;
using aikido::statespace::dart::SO2Joint;
using aikido::trajectory::Interpolated;
using aikido::trajectory::InterpolatedPtr;
using aikido::trajectory::TrajectoryPtr;

namespace {

void checkValidityOfSpaceAndTrajectory(
    const MetaSkeletonStateSpacePtr& space, const InterpolatedPtr trajectory)
{
  if (!space)
    throw std::invalid_argument("StateSpace is null.");

  if (!trajectory)
    throw std::invalid_argument("Trajectory is null.");

  const auto trajectorySpace
      = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
          trajectory->getStateSpace());
  if (!trajectorySpace)
    throw std::invalid_argument(
        "Trajectory is not in a MetaSkeletonStateSpace.");

  // Check that all joints are R1Joint or SO2Joint state spaces.
  for (std::size_t i = 0; i < space->getDimension(); ++i)
  {
    auto jointSpace = space->getJointSpace(i);
    auto properties = jointSpace->getProperties();
    auto r1Joint = std::dynamic_pointer_cast<const R1Joint>(jointSpace);
    auto so2Joint = std::dynamic_pointer_cast<const SO2Joint>(jointSpace);

    if (properties.getNumDofs() != 1 || (!r1Joint && !so2Joint))
    {
      std::stringstream message;
      message << "Only R1Joint and SO2Joint are supported. Joint "
              << properties.getName() << "(index: " << i << ") is a "
              << properties.getType() << " with " << properties.getNumDofs()
              << " DOFs.";
      throw std::invalid_argument{message.str()};
    }
  }
}
}

namespace aikido {
namespace planner {

//==============================================================================
aikido::trajectory::TrajectoryPtr toRevoluteJointTrajectory(
    const MetaSkeletonStateSpacePtr& space, const TrajectoryPtr inputTrajectory)
{
  auto trajectory = std::dynamic_pointer_cast<Interpolated>(inputTrajectory);
  if (!trajectory)
    throw std::invalid_argument("Input trajectory needs to be interpolated");

  checkValidityOfSpaceAndTrajectory(space, trajectory);

  auto interpolator = std::dynamic_pointer_cast<const GeodesicInterpolator>(
      trajectory->getInterpolator());

  // Create new trajectory space.
  std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
  for (std::size_t i = 0; i < space->getDimension(); ++i)
    subspaces.emplace_back(std::make_shared<const R1>());

  auto rSpace = std::make_shared<CartesianProduct>(subspaces);
  auto rInterpolator = std::make_shared<GeodesicInterpolator>(rSpace);
  auto rTrajectory = std::make_shared<Interpolated>(rSpace, rInterpolator);

  Eigen::VectorXd sourceVector(space->getDimension());
  auto sourceState = rSpace->createState();

  // Add the first waypoint
  space->logMap(trajectory->getWaypoint(0), sourceVector);
  rSpace->expMap(sourceVector, sourceState);
  rTrajectory->addWaypoint(0, sourceState);

  auto tangentState = rSpace->createState();
  auto targetState = rSpace->createState();

  // Add the remaining waypoints
  for (std::size_t i = 0; i < trajectory->getNumWaypoints() - 1; ++i)
  {
    const auto tangentVector = interpolator->getTangentVector(
        trajectory->getWaypoint(i), trajectory->getWaypoint(i + 1));

    space->logMap(trajectory->getWaypoint(i), sourceVector);
    rSpace->expMap(sourceVector, sourceState);
    rSpace->expMap(tangentVector, tangentState);
    rSpace->compose(sourceState, tangentState, targetState);

    // Debug
    rSpace->logMap(targetState, sourceVector);
    std::cout << "State is " << sourceVector << std::endl;

    rTrajectory->addWaypoint(i + 1, targetState);
  }

  return rTrajectory;
}

} // namespace planner
} // namespace aikido
