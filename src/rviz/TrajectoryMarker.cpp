#include "aikido/rviz/TrajectoryMarker.hpp"

#include "aikido/rviz/shape_conversions.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace rviz {

//==============================================================================
TrajectoryMarker::TrajectoryMarker(
    interactive_markers::InteractiveMarkerServer* markerServer,
    const std::string& frameId,
    trajectory::ConstTrajectoryPtr trajectory,
    const dart::dynamics::MetaSkeleton& targetSkeleton,
    const dart::dynamics::Frame& frame,
    const Eigen::Vector4d& rgba,
    double thickness,
    std::size_t numLineSegments)
  : mMarkerServer(markerServer)
  , mInteractiveMarker()
  , mFrameId(frameId)
  , mTrajectory(nullptr)
  , mNumLineSegments()
  , mSkeleton(targetSkeleton)
  , mFrame(frame)
  , mNeedUpdate(true)
  , mNeedPointsUpdate(true)
{
  using visualization_msgs::InteractiveMarkerControl;
  using visualization_msgs::Marker;

  // TODO(JS): Check the compatibility of statespaces of the skeleton and the
  // spline

  // Setting invariant properties
  mInteractiveMarker.header.frame_id = mFrameId;
  mInteractiveMarker.name = "Frame[Trajectory]";
  // TODO: Assign trajectory name once available
  mInteractiveMarker.pose.orientation.w = 1;
  mInteractiveMarker.scale = 1;

  mInteractiveMarker.controls.resize(1);
  InteractiveMarkerControl& control = mInteractiveMarker.controls.front();
  control.orientation.w = 1;
  control.orientation_mode = InteractiveMarkerControl::INHERIT;
  control.interaction_mode = InteractiveMarkerControl::NONE;
  control.always_visible = true;
  control.markers.resize(1);

  auto& marker = getMarker();
  marker.type = Marker::LINE_STRIP;
  marker.pose.orientation.w = 1.0;

  // Setting variant properties
  setTrajectory(std::move(trajectory));
  setRGBA(rgba);
  setThickness(thickness);
  setNumLineSegments(numLineSegments);
}

//==============================================================================
TrajectoryMarker::~TrajectoryMarker()
{
  mMarkerServer->erase(mInteractiveMarker.name);
}

//==============================================================================
void TrajectoryMarker::setTrajectory(trajectory::ConstTrajectoryPtr trajectory)
{
  using visualization_msgs::Marker;

  if (trajectory)
  {
    auto statespace = trajectory->getStateSpace();
    if (!std::dynamic_pointer_cast<statespace::dart::MetaSkeletonStateSpace>(
            statespace))
    {
      throw std::invalid_argument(
          "The statespace in the trajectory should be MetaSkeletonStateSpace");
    }
  }

  mTrajectory = std::move(trajectory);

  mNeedPointsUpdate = true;
}

//==============================================================================
trajectory::ConstTrajectoryPtr TrajectoryMarker::getTrajectory() const
{
  return mTrajectory;
}

//==============================================================================
void TrajectoryMarker::setColor(const Eigen::Vector3d& rgb)
{
  auto& marker = getMarker();
  marker.color.r = rgb[0];
  marker.color.g = rgb[1];
  marker.color.b = rgb[2];

  mNeedUpdate = true;
}

//==============================================================================
Eigen::Vector3d TrajectoryMarker::getColor() const
{
  const auto& marker = getMarker();
  return convertROSColorRGBAToEigen(marker.color).head<3>();
}

//==============================================================================
void TrajectoryMarker::setAlpha(double alpha)
{
  auto& marker = getMarker();
  marker.color.a = alpha;

  mNeedUpdate = true;
}

//==============================================================================
double TrajectoryMarker::getAlpha() const
{
  const auto& marker = getMarker();
  return marker.color.a;
}

//==============================================================================
void TrajectoryMarker::setRGBA(const Eigen::Vector4d& rgba)
{
  auto& marker = getMarker();
  marker.color = convertEigenToROSColorRGBA(rgba);

  mNeedUpdate = true;
}

//==============================================================================
Eigen::Vector4d TrajectoryMarker::getRBGA() const
{
  const auto& marker = getMarker();
  return convertROSColorRGBAToEigen(marker.color);
}

//==============================================================================
void TrajectoryMarker::setThickness(double thickness)
{
  auto& marker = getMarker();
  marker.scale.x = thickness;

  mNeedUpdate = true;
}

//==============================================================================
double TrajectoryMarker::getThickness() const
{
  const auto& marker = getMarker();
  return marker.scale.x;
}

//==============================================================================
void TrajectoryMarker::setNumLineSegments(std::size_t numLineSegments)
{
  if (numLineSegments < 2)
  {
    throw std::invalid_argument(
        "Number of line segments should be greater than 2");
  }

  mNumLineSegments = numLineSegments;

  mNeedPointsUpdate = true;
}

//==============================================================================
std::size_t TrajectoryMarker::getNumLineSegments() const
{
  return mNumLineSegments;
}

//==============================================================================
void TrajectoryMarker::update()
{
  if (mNeedPointsUpdate)
    updatePoints();

  if (!mNeedUpdate)
    return;

  mMarkerServer->insert(mInteractiveMarker);

  mNeedUpdate = false;
}

//==============================================================================
void TrajectoryMarker::updatePoints()
{
  using visualization_msgs::Marker;

  if (!mNeedPointsUpdate)
    return;

  auto& marker = getMarker();
  auto& points = marker.points;
  points.clear();

  if (!mTrajectory)
    return;

  Eigen::VectorXd savedPositions = mSkeleton.getPositions();

  statespace::StateSpacePtr statespace = mTrajectory->getStateSpace();
  auto metaSkeletonSs
      = std::dynamic_pointer_cast<statespace::dart::MetaSkeletonStateSpace>(
          statespace);

  auto state = metaSkeletonSs->createState();
  double t = mTrajectory->getStartTime();
  const double dt = mTrajectory->getDuration() / mNumLineSegments;

  points.reserve(mNumLineSegments + 1u);
  Eigen::Vector3d pose;
  for (std::size_t i = 0u; i < mNumLineSegments - 1u; ++i)
  {
    mTrajectory->evaluate(t, state);
    metaSkeletonSs->setState(state);
    pose = mFrame.getTransform().translation();
    points.emplace_back(convertEigenToROSPoint(pose));
    t += dt;
  }
  mTrajectory->evaluate(mTrajectory->getEndTime(), state);
  metaSkeletonSs->setState(state);
  pose = mFrame.getTransform().translation();
  points.emplace_back(convertEigenToROSPoint(pose));

  const_cast<dart::dynamics::MetaSkeleton&>(mSkeleton).setPositions(
      savedPositions);

  mNeedPointsUpdate = false;

  mNeedUpdate = true;
}

//==============================================================================
visualization_msgs::Marker& TrajectoryMarker::getMarker()
{
  using visualization_msgs::Marker;
  Marker& marker = mInteractiveMarker.controls.front().markers.front();
  return marker;
}

//==============================================================================
const visualization_msgs::Marker& TrajectoryMarker::getMarker() const
{
  auto& marker = const_cast<TrajectoryMarker*>(this)->getMarker();
  return const_cast<const visualization_msgs::Marker&>(marker);
}

} // namespace rviz
} // namespace aikido
