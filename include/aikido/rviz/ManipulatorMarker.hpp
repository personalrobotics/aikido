#ifndef AIKIDO_RVIZ_MANIPULATORMARKER_HPP_
#define AIKIDO_RVIZ_MANIPULATORMARKER_HPP_

#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/InverseKinematics.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include "aikido/common/pointers.hpp"
#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace rviz {

AIKIDO_DECLARE_POINTERS(ManipulatorMarker)

/// A wrapper class of RViz InteractiveMarker for visualizing AIKIDO trajectory
/// in RViz.
class ManipulatorMarker final
{
public:
  /// Constructor.
  /// \param[in] markerServer RViz marker server.
  /// \param[in] frameId RViz frame ID.
  /// \param[in] markerName Name of the RViz InteractiveMarker associated with
  /// this TrajectoryMarker. The name must be unique in the same
  /// InteractiveMarkerServer. AIKIDO InteractiveMarkerViewer uses a name
  /// manager for the name uniqueness.
  /// \param[in] skeleton DART MetaSkeleton of the manipulator.
  /// \param[in] bodynode End-Effector of the manipulator to control.
  /// \param[in] collisionConstraint Collision constraints in the environment.
  ManipulatorMarker(
      interactive_markers::InteractiveMarkerServer* markerServer,
      const std::string& frameId,
      const std::string& markerName,
      dart::dynamics::MetaSkeletonPtr skeleton,
      const dart::dynamics::BodyNodePtr bodynode,
      aikido::constraint::TestablePtr collisionConstraint);

  /// Destructor
  ~ManipulatorMarker();

  /// Updates the pose of the manipulator if the marker pose changed.
  void update();

private:
  /// Queries the marker pose to decide if an update is required.
  void getMarkerPose(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback);

  /// Returns marker.
  visualization_msgs::Marker& getMarker();

  /// Returns const marker.
  const visualization_msgs::Marker& getMarker() const;

  /// RViz marker server.
  interactive_markers::InteractiveMarkerServer* mMarkerServer;

  /// RViz interactive marker.
  visualization_msgs::InteractiveMarker mInteractiveMarker;

  /// Frame name of RViz interactive marker.
  std::string mFrameId;

  /// Marker Pose.
  Eigen::Isometry3d mMarkerPose;

  /// DART Metaskeleton.
  dart::dynamics::MetaSkeletonPtr mManipulatorSkeleton;

  /// End-Effector of the manipulator that is controlled with the marker.
  const dart::dynamics::BodyNodePtr mBodyNode;

  /// Inverse Kinematics Solver.
  dart::dynamics::InverseKinematicsPtr mInverseKinematics;

  /// Flag for whether the associated manipulator skeleton needs to be updated.
  bool mNeedUpdate;
};

} // namespace rviz
} // namespace aikido

#endif // AIKIDO_RVIZ_MANIPULATORMARKER_HPP_
