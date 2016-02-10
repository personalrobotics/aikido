#ifndef DART_INTERACTIVEMARKER_SMARTPOINTERS_H_
#define DART_INTERACTIVEMARKER_SMARTPOINTERS_H_
#include <memory>

namespace dart {
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

} // namespace rviz
} // namespace dart

#endif // ifndef DART_INTERACTIVEMARKER_SMARTPOINTERS_H_
