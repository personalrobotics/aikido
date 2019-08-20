#ifndef AIKIDO_IO_TRAJECTORY_HPP_
#define AIKIDO_IO_TRAJECTORY_HPP_

#include "aikido/trajectory/Spline.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace io {

/// Saves a timed trajectory
///
/// Given a spline trajectory \c_traj, object state space and trajectory
/// file path, saves the trajectory as a yaml file for reuse later.
/// \param[in] trajectory Spline trajectory
/// \param[in] skelSpace Metaskeleton of the object
/// \param[in] savePath save path for the trajectory yaml file
void saveTrajectory(const aikido::trajectory::Spline& trajectory,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& skelSpace,
    const std::string& savePath);

/// Load spline trajectory from yaml file
///
/// Given trajectory file and trajectory state space, this method parses
/// the trajectory file and loads a timed trajectory for direct execution.
/// \param[in] trajPath Spline trajectory
/// \param[in] stateSpace Metaskeleton for the trajectory
/// \return Loaded spline trajectory
aikido::trajectory::UniqueSplinePtr loadSplineTrajectory(const std::string& trajPath,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace);
} // namespace io
} // namespace aikido

#endif // AIKIDO_IO_TRAJECTORY_HPP_