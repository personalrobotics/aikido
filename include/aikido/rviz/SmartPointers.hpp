#ifndef AIKIDO_RVIZ_SMARTPOINTERS_H_
#define AIKIDO_RVIZ_SMARTPOINTERS_H_
#include <memory>

namespace aikido {
namespace rviz {

class FrameMarker;
typedef std::shared_ptr<FrameMarker> FrameMarkerPtr;
typedef std::shared_ptr<FrameMarker const> FrameMarkerConstPtr;

class BodyNodeMarker;
typedef std::shared_ptr<BodyNodeMarker> BodyNodeMarkerPtr;
typedef std::shared_ptr<BodyNodeMarker const> BodyNodeMarkerConstPtr;

class SkeletonMarker;
typedef std::shared_ptr<SkeletonMarker> SkeletonMarkerPtr;
typedef std::shared_ptr<SkeletonMarker const> SkeletonMarkerConstPtr;

class TSRMarker;
typedef std::shared_ptr<TSRMarker> TSRMarkerPtr;
typedef std::shared_ptr<TSRMarker const> TSRMarkerConstPtr;

} // namespace rviz
} // namespace aikido

#endif // ifndef AIKIDO_RVIZ_SMARTPOINTERS_H_
