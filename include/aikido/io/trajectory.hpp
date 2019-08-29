#ifndef AIKIDO_IO_TRAJECTORY_HPP_
#define AIKIDO_IO_TRAJECTORY_HPP_

#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Spline.hpp"

/// Format of serialized trajectory in YAML
///
/// configuration:
///   start_time: 0
///   dofs: ["/right/j1", "/right/j2", ...]
///   type: spline
///   spline:
///     order: 3
/// data:
///   - coefficients: [1, 2, ...]
///     duration: 0.1875
///     start_state: [3, 4, ...]
///   - coefficients: [5, 6, ...]
///     duration: 0.1875
///     start_state: [7, 8, ...]

namespace aikido {
namespace io {

/// Serializes a spline trajectory to YAML.
///
/// \param[in] trajectory Spline trajectory
/// \param[in] savePath save path for the trajectory yaml file
void saveTrajectory(
    const aikido::trajectory::Spline& trajectory, const std::string& savePath);

/// Deserializes a spline trajectory from YAML.
///
/// \param[in] trajPath Spline trajectory
/// \param[in] metaSkeletonStateSpace MetaskeletonStateSpace for the trajectory
/// \return Loaded spline trajectory
aikido::trajectory::UniqueSplinePtr loadSplineTrajectory(
    const std::string& trajPath,
    const aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr&
        metaSkeletonStateSpace);

} // namespace io
} // namespace aikido

#endif // AIKIDO_IO_TRAJECTORY_HPP_
