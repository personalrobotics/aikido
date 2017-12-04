#ifndef AIKIDO_TRAJECTORY_TRAJECTORYMETADATA_HPP_
#define AIKIDO_TRAJECTORY_TRAJECTORYMETADATA_HPP_

namespace aikido {
namespace trajectory {

struct TrajectoryMetadata
{
  // TODO: Add more fields from prpy.planning.base.Tags

  /// Whether the trajectory has been validated by an executor, in preparation
  /// for execution.
  bool executorValidated = false;
};

} // namespace trajectory
} // namespace aikido

#endif // AIKIDO_TRAJECTORY_TRAJECTORYMETADATA_HPP_
