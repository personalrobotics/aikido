#include <aikido/planner/Conversions.hpp>

#include <sstream>
#include <aikido/statespace/dart/RnJoint.hpp>
#include <aikido/statespace/dart/SO2Joint.hpp>

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::R1Joint;
using aikido::statespace::dart::SO2Joint;

namespace {

void checkValidityOfSpaceAndTrajectory(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& space,
    const aikido::trajectory::InterpolatedPtr trajectory)
{
  if (!space)
    throw std::invalid_argument("StateSpace is null.");

  if (!trajectory)
    throw std::invalid_argument("Trajectory is null.");

  const auto trajectorySpace = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
      trajectory->getStateSpace());
  if (!trajectorySpace)
    throw std::invalid_argument("Trajectory is not in a MetaSkeletonStateSpace.");

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
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& space,
    const aikido::trajectory::InterpolatedPtr trajectory)
{
  checkValidityOfSpaceAndTrajectory(space, trajectory);

  // Get the right trajectory.
  std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
  for (std::size_t i = 0; i < space->getDimension(); ++i)
    subspaces.emplace_back(std::make_shared<const aikido::statespace::R1>());

  auto interpolator = trajectory->getInterpolator();

  auto rSpace = std::make_shared<aikido::statespace::CartesianProduct>(subspaces);
  auto rInterpolator = std::make_shared<statespace::GeodesicInterpolator>(rSpace);
  auto rTrajectory = std::make_shared<trajectory::Interpolated>(rSpace, rInterpolator);



  // Interpolate appropriately with

  return rTrajectory;
}

} // namespace planner
} // namespace aikido
