#ifndef AIKIDO_RVIZ_TSRMARKER_H_
#define AIKIDO_RVIZ_TSRMARKER_H_

#include <set>
#include "FrameMarker.hpp"
#include "SmartPointers.hpp"

namespace aikido {
namespace rviz {

class TSRMarker {
public:
  TSRMarker();
  ~TSRMarker(){};

  void addFrameMarker(FrameMarkerPtr const &marker);

private:
  std::set<FrameMarkerPtr> mFrameMarkers;

};

} // namespace rviz
} // namespace aikido

#endif // ifndef AIKIDO_RVIZ_TSRMARKER_H_
