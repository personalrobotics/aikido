#include "aikido/rviz/TrajectoryMarker.hpp"

#include "aikido/rviz/shape_conversions.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace rviz {

//==============================================================================
TrajectoryMarker::TrajectoryMarker(
    interactive_markers::InteractiveMarkerServer* markerServer,
    const std::string& frameId,
    const std::string& markerName,
    trajectory::ConstTrajectoryPtr trajectory,
    dart::dynamics::MetaSkeletonPtr targetSkeleton,
    const dart::dynamics::Frame& frame,
    const Eigen::Vector4d& rgba,
    double thickness,
    std::size_t numLineSegments)
  : mMarkerServer(markerServer)
  , mInteractiveMarker()
  , mFrameId(frameId)
  , mTrajectory(nullptr)
  , mNumLineSegments()
  , mSkeleton(std::move(targetSkeleton))
  , mFrame(frame)
  , mNeedUpdate(true)
  , mNeedPointsUpdate(true)
{
  using visualization_msgs::InteractiveMarkerControl;
  using visualization_msgs::Marker;

  // Setting invariant properties
  mInteractiveMarker.header.frame_id = mFrameId;
  mInteractiveMarker.name = markerName;
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
    if (!std::dynamic_pointer_cast<const statespace::dart::
                                       MetaSkeletonStateSpace>(statespace))
    {
      throw std::invalid_argument(
          "The statespace in the trajectory should be MetaSkeletonStateSpace");
    }

    if (statespace->getDimension() != mSkeleton->getNumDofs())
    {
      throw std::invalid_argument(
          "The statespace in the trajectory is not compatible (dimensions are "
          "different) to DART meta skeleton for visualizing the trajectory");
    }
    // TODO: Use more comprehensive compatibility check once available than
    // just checking the dimensions.
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
  if (numLineSegments == 0u)
  {
    dtwarn << "[TrajectoryMarker::setNumLineSegments] numLineSegments is set to"
              "zero. This trajectory will not be rendered.";
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

  if (!mTrajectory || mNumLineSegments == 0u)
  {
    mNeedPointsUpdate = false;
    mNeedUpdate = true;
    return;
  }

  auto statespace = mTrajectory->getStateSpace();
  auto metaSkeletonSs = std::
      dynamic_pointer_cast<const statespace::dart::MetaSkeletonStateSpace>(
          statespace);

  auto saver = statespace::dart::MetaSkeletonStateSaver(mSkeleton);
  DART_UNUSED(saver);

  auto state = metaSkeletonSs->createState();
  double t = mTrajectory->getStartTime();
  const double dt = mTrajectory->getDuration() / mNumLineSegments;

  points.reserve(mNumLineSegments + 1u);
  Eigen::Vector3d pose;
  assert(mNumLineSegments != 0u);
  for (std::size_t i = 0u; i < mNumLineSegments - 1u; ++i)
  {
    mTrajectory->evaluate(t, state);
    metaSkeletonSs->setState(mSkeleton.get(), state);
    pose = mFrame.getTransform().translation();
    points.emplace_back(convertEigenToROSPoint(pose));
    t += dt;
  }
  mTrajectory->evaluate(mTrajectory->getEndTime(), state);
  metaSkeletonSs->setState(mSkeleton.get(), state);
  pose = mFrame.getTransform().translation();
  points.emplace_back(convertEigenToROSPoint(pose));

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
