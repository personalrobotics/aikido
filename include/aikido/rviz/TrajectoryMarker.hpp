#ifndef AIKIDO_RVIZ_TRAJECTORYMARKER_HPP_
#define AIKIDO_RVIZ_TRAJECTORYMARKER_HPP_

#include <dart/dynamics/Frame.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace rviz {

/// A wrapper class of RViz InteractiveMarker for visualizing AIKIDO trajectory
/// in RViz.
class TrajectoryMarker final
{
public:
  /// Constructor.
  /// \param[in] markerServer RViz marker server.
  /// \param[in] frameId RViz frame ID.
  /// \param[in] trajectory C-sapce (or joint-space) trajectory.
  /// \param[in] skeleton Target DART meta skeleton that the C-space trajectory
  /// will be applied to compute the visualizing task-space trajectory.
  /// \param[in] frame Target DART frame where the trajectory of its origin
  /// will be visualized.
  /// \param[in] rgba Color and alpha of the visualizing task-space trajectory.
  /// Default is [RGBA: 0.75, 0.75, 0.75, 0.75].
  /// \param[in] thickness Thickness of the visualizing task-space trajectory.
  /// Default is 0.01.
  /// \param[in] numLineSegments Number of line-segments of the visualizing
  /// task-space trajectory. Default is 16.
  TrajectoryMarker(
      interactive_markers::InteractiveMarkerServer* markerServer,
      const std::string& frameId,
      trajectory::ConstTrajectoryPtr trajectory,
      const dart::dynamics::MetaSkeleton& skeleton,
      const dart::dynamics::Frame& frame,
      const Eigen::Vector4d& rgba = Eigen::Vector4d::Constant(0.75),
      double thickness = 0.01,
      std::size_t numLineSegments = 16u);

  /// Destructor
  ~TrajectoryMarker();

  /// Sets or updates trajectory to visualize
  void setTrajectory(trajectory::ConstTrajectoryPtr trajectory);

  /// Returns trajectory associated with this marker
  trajectory::ConstTrajectoryPtr getTrajectory() const;

  /// Sets or updates color (RGB) of visualizing trajectory.
  void setColor(const Eigen::Vector3d& rgb);

  /// Returns color (RGB) of visualizing trajectory.
  Eigen::Vector3d getColor() const;

  /// Sets or updates alpha of visualizing trajectory.
  void setAlpha(double alpha);

  /// Returns alpha of visualizing trajectory.
  double getAlpha() const;

  /// Sets or updates RGBA of visualizing trajectory.
  void setRGBA(const Eigen::Vector4d& rgb);

  /// Returns RGBA of visualizing trajectory.
  Eigen::Vector4d getRBGA() const;

  /// Sets or updates thickness of visualizing trajectory.
  void setThickness(double thickness);

  /// Returns thickness of visualizing trajectory.
  double getThickness() const;

  /// Sets or updates number of line segments of the visualizing trajectory.
  void setNumLineSegments(std::size_t numLineSegments);

  /// Returns number of line segments of the visualizing trajectory.
  std::size_t getNumLineSegments() const;

  /// Updates this marker.
  ///
  /// This function should be called after the properties are changed (e.g.,
  /// trajectory, color, thickness, number of line-segments) so that RViz
  /// reflects the changes accordingly.
  void update();

private:
  /// Updates trajectory points based on trajectory and number of line-segments.
  ///
  /// This function is called by update().
  void updatePoints();

  /// Returns marker.
  visualization_msgs::Marker& getMarker();

  /// Returns const marker.
  const visualization_msgs::Marker& getMarker() const;

  /// RViz marker server.
  interactive_markers::InteractiveMarkerServer* mMarkerServer;

  /// RViz interactive marker.
  visualization_msgs::InteractiveMarker mInteractiveMarker;

  ///
  std::string mFrameId;

  /// C-sapce (or joint-space) trajectory.
  trajectory::ConstTrajectoryPtr mTrajectory;

  /// Number of line segments of the discretized task-space trajectory.
  std::size_t mNumLineSegments;
  // TODO(JS): Consider switching to resolution once arc-length calculation is
  // available for Trajectoy.

  /// Target DART meta skeleton that the C-space trajectory will be applied to
  /// compute the visualizing task-space trajectory.
  const dart::dynamics::MetaSkeleton& mSkeleton;

  /// Target DART frame where the trajectory of its origin will be visualized.
  const dart::dynamics::Frame& mFrame;

  /// Whether the associated RViz maker needs to be updated.
  bool mNeedUpdate;

  /// Whether the trajectory points need to be updated.
  bool mNeedPointsUpdate;
};

} // namespace rviz
} // namespace aikido

#endif // AIKIDO_RVIZ_TRAJECTORYMARKER_HPP_
