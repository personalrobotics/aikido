#ifndef AIKIDO_PERCEPTION_PERCEPTIONMODULE_HPP_
#define AIKIDO_PERCEPTION_PERCEPTIONMODULE_HPP_

#include <aikido/planner/World.hpp>

namespace aikido {
namespace perception {

/// The interface for the generic perception module. Provides a \c
/// detectObjects() method for detecting all objects in the environment and
/// updating the world representation accordingly.
///
class PerceptionModule
{
public:
  virtual ~PerceptionModule() = default;

  /// Run the specific detector via a service call or ROS message reception,
  /// lookup the transform between the specified frames, load object models for
  /// new objects, compute their pose, and update the list of DART Skeletons
  /// that represents the world.
  ///
  /// \param[in,out] skeleton_list The set of skeletons currently in context. It
  /// will either be added to or updated.
  /// \param[in] timeout The duration up to which to wait for the transform.
  /// Returns false if none of the markers get correctly transformed
  /// \param[in] timestamp Only detections more recent than this timestamp will
  /// be accepted. A timestamp of 0 greedily takes the first available message,
  /// and is the default behaviour.
  /// \return Returns \c false if no detection observed, or if none of the
  /// detections has a more recent timestamp than the parameter. Returns \c true
  /// otherwise.
  virtual bool detectObjects(
      const aikido::planner::WorldPtr& env,
      ros::Duration timeout,
      ros::Time timestamp)
      = 0;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_PERCEPTIONMODULE_HPP_
